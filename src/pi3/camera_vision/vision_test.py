"""Camera vision: red block, green block and the black wall.

For each thing it sees, it reports coordinates, which side of the frame it is on,
and how far away it is.

Cost per frame is O(N) in pixels + O(C) in contours found, and N is fixed by
PROC_WIDTH rather than the camera resolution. Nothing here sorts, rescans a mask,
or recomputes a contour property it already has.

Run:  python vision_test.py
      python vision_test.py --selftest    (no camera needed)

Keys: Q quit
      M toggle the mask view, which is how the HSV ranges get tuned
      D toggle a half size window, which is much cheaper to draw on a Pi 3
      P print the HSV value under the crosshair, for setting thresholds

The readout in the corner separates processing time from total loop time. On a
Pi 3 the processing is a few milliseconds and drawing the window is most of the
rest, which is why the competition program does not open one.
"""

import sys
import time
from math import radians, tan

import cv2
import numpy as np

# --- Camera -----------------------------------------------------------------
CAMERA_INDEX = 0        # laptop webcam. Pi Camera via libcamera also shows up as 0.
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
CAMERA_HFOV_DEG = 62.2  # Pi Camera v2. CALIBRATE: put a block at a measured 50 cm
                        # and adjust until the distance readout agrees.

# --- Detection --------------------------------------------------------------
# Blocks stand on the mat, so the top of the frame is ceiling and lights - only
# noise lives up there. Cropping it costs nothing and removes false positives.
ROI_TOP = 0.35          # fraction of the frame ignored from the top
PROC_WIDTH = 320        # detection runs on a downscaled copy; drawing stays full size
DISPLAY_SCALE = 0.5     # window size relative to the frame, for the D key
PROBE_PATCH = 12        # pixels either side of centre sampled by the P key

MIN_AREA = 150          # px^2 at PROC_WIDTH. Small, so a far block still registers
MIN_RATIO, MAX_RATIO = 0.15, 2.00   # w/h. Blocks are taller than wide, even clipped
MIN_FILL = 0.45         # contour area / bounding box area. A solid block mostly fills
                        # its box; scattered glare or carpet texture does not.
WALL_MIN_AREA = 600     # the wall is a big object, so it can afford a high floor

# Side bands used to judge lane position from the camera. Only the outer thirds
# of the frame are counted, because the middle contains the wall ahead and any
# traffic signs, neither of which say anything about where we sit across the lane.
WALL_BAND_WIDTH = 0.30  # fraction of the processed width each band covers
CENTRE_DEADBAND = 0.12  # |offset| below this counts as straight ahead

BLOCK_HEIGHT_CM = 10.0  # WRO signal block
WALL_HEIGHT_CM = 10.0   # mat wall

# HSV ranges. Red wraps around hue 0, so it needs two ranges.
#
# Saturation and value do different jobs, and getting them the wrong way round
# was a real bug. Putting an object in shadow lowers its VALUE but leaves its
# SATURATION almost untouched, because saturation describes how pure the colour
# is rather than how bright. So:
#
#   value floor stays LOW   - a block in shadow must still be found
#   saturation floor stays HIGH - this is what separates a painted block from
#                                 skin, cardboard and anything else vaguely warm
#
# Skin measures around hue 10 to 15 with saturation near 100. A printed traffic
# sign sits well above 180. The floor below is set between the two.
COLOUR_RANGES = {
    "RED": [((0, 150, 40), (10, 255, 255)), ((172, 150, 40), (180, 255, 255))],
    "GREEN": [((35, 120, 35), (90, 255, 255))],
}

# The parking markers are magenta, which sits immediately below red on the hue
# circle - close enough that the old upper-red band (165+) swallowed it. Red now
# starts at 172 instead, which costs nothing because red also has the 0-10 band.
MAGENTA_RANGE = [((140, 120, 60), (170, 255, 255))]
MARKER_MIN_AREA = 120       # px^2 at PROC_WIDTH. Markers are seen from far off.
MARKER_HEIGHT_CM = 10.0     # CHECK against the rulebook and a ruler
# The wall is black, and black means dark AND colourless. The saturation ceiling
# is what stops every dark object in the room registering as track: dark clothing
# measures around saturation 150, a black wall around 20. Without that ceiling a
# navy jumper at the edge of frame corrupts the lane measurement.
#
# The floor of the track is white, so there is a wide gap between wall and floor
# in value, which is why this threshold can afford to be strict.
WALL_RANGE = [((0, 0, 0), (180, 70, 70))]

DRAW_COLOUR = {"RED": (0, 0, 255), "GREEN": (0, 255, 0), "WALL": (255, 200, 0),
               "PARKING": (255, 0, 255)}

KERNEL = np.ones((5, 5), np.uint8)
TAN_HALF_HFOV = tan(radians(CAMERA_HFOV_DEG) / 2)   # hoisted: same every frame


def colour_mask(hsv, ranges):
    """Binary mask for one colour, cleaned of speckle and pinholes. O(N)."""
    mask = cv2.inRange(hsv, np.array(ranges[0][0], np.uint8),
                       np.array(ranges[0][1], np.uint8))
    for low, high in ranges[1:]:            # only red needs a second pass
        mask |= cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)    # kill speckle
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)   # fill glare holes


def find_block(mask):
    """Biggest block-shaped blob: returns (x, y, w, h) or None.

    One pass, keeping the best so far - no sort, and each contour's area is
    computed exactly once. Shape matters as much as size: area alone happily
    locks onto a red jacket or a strip of glare on the mat.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_area = None, MIN_AREA        # doubles as the minimum-area filter
    for contour in contours:
        area = cv2.contourArea(contour)
        if area <= best_area:
            continue                        # too small, or worse than what we have
        x, y, w, h = cv2.boundingRect(contour)
        if not MIN_RATIO <= w / h <= MAX_RATIO:
            continue
        if area / (w * h) < MIN_FILL:
            continue
        best, best_area = (x, y, w, h), area
    return best


def wall_bands(wall_mask):
    """How much wall fills each side of the frame, and the balance between them.

    Returns (left, right, balance). Balance runs -1 to +1, and positive means
    more wall on the left, so the robot has drifted towards the left wall and
    should steer right.

    This is a lane position measurement taken from the camera rather than from
    the distance sensors. It costs one pixel count per side, it arrives at frame
    rate rather than at the sensor sweep rate, and because the camera looks
    ahead it starts responding to a curve before the robot reaches it.

    Dividing by the total is what makes it independent of resolution and of how
    brightly the walls happen to be lit.
    """
    height, width = wall_mask.shape
    band = int(width * WALL_BAND_WIDTH)
    left = cv2.countNonZero(wall_mask[:, :band])
    right = cv2.countNonZero(wall_mask[:, width - band:])
    total = left + right
    return left, right, 0.0 if total == 0 else (left - right) / total


def find_wall(mask):
    """Biggest dark region. Same single pass, but no shape test - a wall is a
    long low band, nothing like a block."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_area = None, WALL_MIN_AREA
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > best_area:
            best, best_area = cv2.boundingRect(contour), area
    return best


def find_markers(mask):
    """The two largest magenta regions, left to right. One pass, like find_block.

    No shape test: a parking marker is a low wide barrier, nothing like a
    pillar, and it is often clipped by the edge of the frame.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = []
    for contour in contours:
        if cv2.contourArea(contour) >= MARKER_MIN_AREA:
            found.append(cv2.boundingRect(contour))
    if not found:
        return []
    # biggest two, then back into left-to-right order
    found.sort(key=lambda box: box[2] * box[3], reverse=True)
    return sorted(found[:2], key=lambda box: box[0])


def parking_gap(markers, scale, roi_y, width):
    """Where to aim to park, from the markers we can see.

    With both markers the target is the middle of the gap between their inner
    edges - that is the slot itself, not either marker. With only one there is
    no gap yet, so we report the marker and let the caller keep looking.
    """
    if not markers:
        return None

    boxes = [(int(x * scale), int(y * scale) + roi_y, int(w * scale), int(h * scale))
             for x, y, w, h in markers]
    tallest = max(box[3] for box in boxes)

    if len(boxes) == 2:
        left, right = boxes
        gap_x = (left[0] + left[2] + right[0]) // 2     # inner edge to inner edge
        gap_width = right[0] - (left[0] + left[2])
        span = (left[0], min(left[1], right[1]),
                right[0] + right[2] - left[0], max(left[3], right[3]))
    else:
        box = boxes[0]
        gap_x = box[0] + box[2] // 2
        gap_width = 0
        span = box

    focal_px = (width / 2) / TAN_HALF_HFOV
    offset, position = direction(gap_x, width)
    return {"box": span, "cx": gap_x, "cy": span[1] + span[3] // 2,
            "offset": offset, "position": position,
            "distance": MARKER_HEIGHT_CM * focal_px / max(tallest, 1),
            "markers": len(boxes), "gap_px": gap_width}


def direction(cx, width):
    """Where something sits: offset in -1..1 (left..right) plus a label."""
    offset = (cx - width / 2) / (width / 2)
    if offset < -CENTRE_DEADBAND:
        return offset, "LEFT"
    if offset > CENTRE_DEADBAND:
        return offset, "RIGHT"
    return offset, "CENTRE"


def describe(box, scale, roi_y, width, real_height_cm):
    """Scale a detection back to full-frame coordinates and measure it.

    Distance is the pinhole estimate from the object's known real height against
    its pixel height. Nearer object -> taller in frame -> smaller distance.
    """
    x, y, w, h = (int(v * scale) for v in box)
    y += roi_y
    cx, cy = x + w // 2, y + h // 2
    offset, position = direction(cx, width)
    focal_px = (width / 2) / TAN_HALF_HFOV
    return {"box": (x, y, w, h), "cx": cx, "cy": cy, "offset": offset,
            "position": position, "distance": real_height_cm * focal_px / max(h, 1)}


def detect(frame):
    """Everything visible this frame.

    Returns (seen, masks, walls):
      seen   name -> dict of box/cx/cy/offset/position/distance
      masks  name -> binary mask, for the mask view
      walls  dict of left/right wall pixels in the side bands, and the balance
             between them, which is how lane position is judged from the camera
    """
    height, width = frame.shape[:2]
    roi_y = int(height * ROI_TOP)
    proc_height = int((height - roi_y) * PROC_WIDTH / width)
    small = cv2.resize(frame[roi_y:], (PROC_WIDTH, proc_height),
                       interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)    # one conversion, reused below
    scale = width / PROC_WIDTH

    seen, masks = {}, {}
    for name, ranges in COLOUR_RANGES.items():
        masks[name] = colour_mask(hsv, ranges)
        found = find_block(masks[name])
        if found:
            seen[name] = describe(found, scale, roi_y, width, BLOCK_HEIGHT_CM)

    masks["WALL"] = colour_mask(hsv, WALL_RANGE)
    found = find_wall(masks["WALL"])
    if found:
        seen["WALL"] = describe(found, scale, roi_y, width, WALL_HEIGHT_CM)

    masks["PARKING"] = colour_mask(hsv, MAGENTA_RANGE)
    parking = parking_gap(find_markers(masks["PARKING"]), scale, roi_y, width)
    if parking:
        seen["PARKING"] = parking

    left, right, balance = wall_bands(masks["WALL"])
    walls = {"left": left, "right": right, "balance": balance}
    return seen, masks, walls


def draw(frame, name, info):
    """Full box, centre, coordinates and distance."""
    x, y, w, h = info["box"]
    colour = DRAW_COLOUR[name]

    patch = frame[y:y + h, x:x + w]
    cv2.addWeighted(patch, 0.72, np.full_like(patch, colour), 0.28, 0, patch)
    cv2.rectangle(frame, (x, y), (x + w, y + h), colour, 3)
    cv2.drawMarker(frame, (info["cx"], info["cy"]), (255, 255, 255),
                   cv2.MARKER_CROSS, 14, 2)

    label = f"{name} {info['position']}"
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
    bar_y = max(y - th - 12, 0)
    cv2.rectangle(frame, (x, bar_y), (x + tw + 10, bar_y + th + 10), colour, -1)
    cv2.putText(frame, label, (x + 5, bar_y + th + 3),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, f"({info['cx']},{info['cy']}) {info['distance']:.0f}cm",
                (x, y + h + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)


def probe_hsv(frame):
    """Average HSV of a small patch at the centre of the frame.

    Point the crosshair at a real traffic sign on the real mat and press P. The
    numbers printed are what the thresholds have to accept. Saturation is the
    one that matters most: it is what separates a painted sign from skin or
    cardboard, and unlike value it barely changes when the sign is in shadow.
    """
    height, width = frame.shape[:2]
    cy, cx = height // 2, width // 2
    patch = frame[cy - PROBE_PATCH:cy + PROBE_PATCH,
                  cx - PROBE_PATCH:cx + PROBE_PATCH]
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    return [int(round(v)) for v in cv2.mean(hsv)[:3]]


def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # newest frame, not the oldest queued one
    if not cap.isOpened():
        sys.exit(f"Could not open camera {CAMERA_INDEX}")

    show_mask = False
    small_window = False
    detect_ms = loop_ms = 0.0
    last_loop = time.perf_counter()

    print("Q quit   M mask view   D half size window   P probe HSV at centre")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Dropped frame")
            continue

        height, width = frame.shape[:2]
        roi_y = int(height * ROI_TOP)

        started = time.perf_counter()
        seen, masks, walls = detect(frame)
        detect_ms = 0.8 * detect_ms + 0.2 * (time.perf_counter() - started) * 1000

        if show_mask:
            combined = (masks["RED"] | masks["GREEN"] | masks["WALL"]
                        | masks["PARKING"])
            frame[roi_y:] = cv2.resize(cv2.cvtColor(combined, cv2.COLOR_GRAY2BGR),
                                       (width, height - roi_y))

        for name, info in seen.items():
            draw(frame, name, info)

        cv2.line(frame, (width // 2, roi_y), (width // 2, height), (200, 200, 200), 1)
        cv2.line(frame, (0, roi_y), (width, roi_y), (200, 200, 200), 1)
        cv2.putText(frame, "wall L%d R%d  balance %+.2f"
                    % (walls["left"], walls["right"], walls["balance"]),
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        # a bar showing which way the wall balance is pushing the steering
        centre = width // 2
        cv2.line(frame, (centre, 40), (centre + int(walls["balance"] * 120), 40),
                 (255, 200, 0), 6)

        # Processing time against total loop time. If these differ a lot, the
        # window is the cost, not the vision.
        cv2.putText(frame, "detect %.1f ms   loop %.0f ms   %.0f fps"
                    % (detect_ms, loop_ms, 1000 / max(loop_ms, 1e-6)),
                    (10, height - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 2)
        cv2.drawMarker(frame, (width // 2, height // 2), (0, 255, 255),
                       cv2.MARKER_CROSS, 18, 1)

        cv2.imshow("vision test",
                   cv2.resize(frame, None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE)
                   if small_window else frame)

        now = time.perf_counter()
        loop_ms = 0.8 * loop_ms + 0.2 * (now - last_loop) * 1000
        last_loop = now

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("m"):
            show_mask = not show_mask
        if key == ord("d"):
            small_window = not small_window
        if key == ord("p"):
            hue, sat, val = probe_hsv(frame)
            print("centre HSV  H %3d  S %3d  V %3d" % (hue, sat, val))

    cap.release()
    cv2.destroyAllWindows()


def selftest():
    """Synthetic mat: grey floor, dark wall band, both blocks visible at once."""
    frame = np.full((480, 640, 3), 150, np.uint8)               # light mat
    cv2.rectangle(frame, (0, 180), (639, 235), (25, 25, 25), -1)  # black wall
    cv2.rectangle(frame, (100, 250), (160, 430), (0, 0, 255), -1)  # red, left, near
    cv2.rectangle(frame, (420, 300), (470, 430), (0, 255, 0), -1)  # green, right, far
    cv2.rectangle(frame, (300, 300), (308, 306), (0, 0, 255), -1)  # speckle, ignored

    seen, masks, walls = detect(frame)

    # all three at once - that is the whole point
    assert set(seen) == {"RED", "GREEN", "WALL"}, sorted(seen)
    assert abs(seen["RED"]["cx"] - 130) < 8, seen["RED"]
    assert abs(seen["GREEN"]["cx"] - 445) < 8, seen["GREEN"]
    assert seen["RED"]["position"] == "LEFT"
    assert seen["GREEN"]["position"] == "RIGHT"

    # the taller block is the nearer one
    assert seen["RED"]["distance"] < seen["GREEN"]["distance"]
    assert 20 < seen["RED"]["distance"] < 40, seen["RED"]["distance"]

    # the wall spans the frame evenly, so the balance is near zero
    assert abs(walls["balance"]) < 0.2, walls

    # --- lane position from the camera ---
    # wall only down the left side: we have drifted left, so steer right
    left_hug = np.full((480, 640, 3), 150, np.uint8)
    cv2.rectangle(left_hug, (0, 200), (120, 479), (25, 25, 25), -1)
    assert detect(left_hug)[2]["balance"] > 0.8, detect(left_hug)[2]

    # and the mirror image
    right_hug = np.full((480, 640, 3), 150, np.uint8)
    cv2.rectangle(right_hug, (519, 200), (639, 479), (25, 25, 25), -1)
    assert detect(right_hug)[2]["balance"] < -0.8, detect(right_hug)[2]

    # centred between two walls reads as balanced
    centred = np.full((480, 640, 3), 150, np.uint8)
    cv2.rectangle(centred, (0, 200), (90, 479), (25, 25, 25), -1)
    cv2.rectangle(centred, (549, 200), (639, 479), (25, 25, 25), -1)
    assert abs(detect(centred)[2]["balance"]) < 0.1, detect(centred)[2]

    # no wall in view at all gives no signal rather than a wrong one
    assert detect(np.full((480, 640, 3), 150, np.uint8))[2]["balance"] == 0.0

    # nothing but mat -> nothing seen
    empty, _, _ = detect(np.full((480, 640, 3), 150, np.uint8))
    assert empty == {}, empty

    assert direction(320, 640)[1] == "CENTRE"
    print("selftest ok")


if __name__ == "__main__":
    selftest() if "--selftest" in sys.argv else main()
