"""Navigation Engine: vision + robot state -> one movement command.

    compute_navigation(vision_data, robot_state) -> {"speed", "steering", ...}

Decides only. It never touches a motor or a servo - the command it returns goes
over UART to the Pico, which owns the hardware.

Sign convention, the same one the UART link and the Pico use:
    steering  positive = right, negative = left, 0 = straight
    speed     0 = stopped, 100 = flat out

The steering maths is pure: every function below takes what it needs as an
argument and reads nothing but configuration constants. The only state in the
file is the previous frame's steering and speed mode, held by compute_navigation
so that rate limiting and hysteresis have something to compare against.

Run:  python navigation_engine.py            (demo of the decision table)
      python navigation_engine.py --selftest
"""

import sys

# ============================================================================
# CONFIGURATION - every tunable number in the file lives here
# ============================================================================

# --- Colours ----------------------------------------------------------------
COLOUR_RED = "RED"
COLOUR_GREEN = "GREEN"

# --- Camera -----------------------------------------------------------------
CAMERA_WIDTH = 640
CAMERA_CENTRE_X = CAMERA_WIDTH // 2
MIN_PILLAR_AREA = 500           # px^2. Below this it is noise, not a pillar

# --- Steering ---------------------------------------------------------------
MAX_STEERING_DEG = 30           # matches the servo's mechanical limit
STEERING_GAIN = 30.0            # degrees of steering per unit of offset error
STRAIGHT_STEERING_DEG = 0

# Where a pillar should sit in the frame once we are safely past it. Passing a
# red pillar on its RIGHT means the pillar ends up on our LEFT, i.e. negative.
PASS_TARGET_OFFSET = 0.5
PASS_TARGET = {COLOUR_RED: -PASS_TARGET_OFFSET, COLOUR_GREEN: PASS_TARGET_OFFSET}
# Which way we are allowed to steer for each colour. Red is passed on the right,
# so a red pillar may only ever produce right steering, and vice versa.
PASS_DIRECTION = {COLOUR_RED: 1, COLOUR_GREEN: -1}

# Jitter of a few pixels around the centre should not keep nudging the wheels,
# so an offset this small is treated as exactly centred.
CENTRE_DEAD_ZONE = 0.06
# How far off centre before we call it left or right in the debug output.
POSITION_CENTRE_BAND = 0.15
# Most the steering angle may change between consecutive frames. Lower is
# smoother but slower to react.
STEERING_MAX_CHANGE_DEG = 6

# --- Lane keeping -----------------------------------------------------------
# The left and right ToF sensors sit at +/-45 degrees either side of the camera,
# so they watch the forward diagonals. Comparing the two tells us which side of
# the lane we are on without ever needing to know how wide the lane is.
LANE_GAIN = 22.0            # degrees of steering per unit of lane offset

# Lane position from the camera. Preferred over the distance sensors because it
# arrives with the frame instead of at the end of a 132 ms sensor sweep, and
# because the camera sees ahead, so it starts correcting for a curve before the
# robot reaches it. The sensors remain the fallback when no wall is in view.
WALL_BALANCE_GAIN = 26.0
WALL_BAND_MIN_PIXELS = 60   # below this there is not enough wall to steer by
LANE_DEAD_ZONE = 0.08       # closer to centre than this counts as centred
LANE_MAX_VALID_MM = 2000    # a VL53L0X reports ~8190 mm when it sees nothing

# Modes that change which steering law applies. Anything else follows the
# pillar when one is visible and the lane when one is not.
MODE_TURN_CORNER = "TURN_CORNER"
MODE_RECENTER = "RECENTER"

# --- Missions ---------------------------------------------------------------
# These strings match mission_manager.Mission. They are repeated rather than
# imported so navigation stays free of any dependency on it - the mission
# arrives as an argument, like everything else this module reasons about.
MISSION_OPEN = "OPEN_CHALLENGE"
MISSION_OBSTACLE = "OBSTACLE_CHALLENGE"
MISSION_PARKING = "PARKING"

# Only the obstacle round has pillars to dodge. In the others a red or green
# object is scenery, and swerving at it would be a mistake.
MISSIONS_WITH_PILLARS = (MISSION_OBSTACLE,)

# Behaviours (state machine states) during which we are working a pillar.
MODE_APPROACH_PILLAR = "APPROACH_PILLAR"
MODE_PASS_PILLAR = "PASS_PILLAR"
PILLAR_MODES = (MODE_APPROACH_PILLAR, MODE_PASS_PILLAR)

# Parking happens at the end of three laps, back where the robot started.
MODE_SEARCH_PARKING = "SEARCH_PARKING"
MODE_ALIGN_PARKING = "ALIGN_PARKING"
MODE_ENTER_PARKING = "ENTER_PARKING"
PARKING_MODES = (MODE_SEARCH_PARKING, MODE_ALIGN_PARKING, MODE_ENTER_PARKING)

PARKING_GAIN = 25.0             # degrees of steering per unit of gap offset

# --- Speed ------------------------------------------------------------------
CRUISE_SPEED = 45
SLOW_SPEED = 25
STOP_SPEED = 0

# Cruising speed per mission. The open round has nothing to dodge, so it can
# run harder; parking is a crawl from start to finish.
MISSION_CRUISE_SPEED = {
    MISSION_OPEN: 60,
    MISSION_OBSTACLE: CRUISE_SPEED,
    MISSION_PARKING: 20,
}
PILLAR_APPROACH_SPEED = 30      # slow down while working a pillar

MODE_CRUISE = "CRUISE"
MODE_SLOW = "SLOW"
MODE_STOP = "STOP"

# Hysteresis: each threshold has a lower value to enter the slower state and a
# higher one to leave it. A reading hovering on a single threshold would
# otherwise flip the robot between cruise and slow several times a second.
SLOW_ENTER_MM = 400             # closer than this -> slow down
SLOW_EXIT_MM = 500              # must be clearer than this to speed up again
STOP_ENTER_MM = 150             # closer than this -> stop
STOP_EXIT_MM = 220              # must be clearer than this to move off again

DEBUG = False

# ============================================================================
# PURE FUNCTIONS - no state, no globals beyond the configuration above
# ============================================================================


def normalise_pillars(vision_data):
    """Accept whichever shape the vision module hands over.

    Preferred is a list of pillars, so several can be visible at once:
        {"pillars": [{"colour": "RED", "x": 410, "area": 8200}, ...]}
    A single-pillar dict with largest_colour/center_x/area still works.
    """
    if not vision_data:
        return []
    if isinstance(vision_data, list):
        return vision_data
    if vision_data.get("pillars"):
        return vision_data["pillars"]

    colour = vision_data.get("largest_colour")
    if not colour:
        return []
    return [{"colour": colour,
             "x": vision_data.get("center_x", CAMERA_CENTRE_X),
             "area": vision_data.get("area", 0),
             "distance": vision_data.get("distance")}]


def select_pillar(pillars):
    """The one pillar that matters this frame, or None.

    Both colours are often in view at once - a red one close and a green one
    further down the track, say. The near one is the one about to be hit, so
    that is the one we steer around; the far one is dealt with on later frames
    once it becomes the near one.
    """
    candidates = [p for p in pillars
                  if p.get("colour") in PASS_TARGET
                  and p.get("area", 0) >= MIN_PILLAR_AREA]
    if not candidates:
        return None

    # Use measured distance when vision provides it. Otherwise fall back to
    # apparent size, since a nearer pillar of the same real size looks bigger.
    if all(p.get("distance") is not None for p in candidates):
        return min(candidates, key=lambda p: p["distance"])
    return max(candidates, key=lambda p: p["area"])


def pillar_offset(centre_x):
    """Horizontal position in the frame as -1 (left edge) to +1 (right edge)."""
    offset = (centre_x - CAMERA_CENTRE_X) / CAMERA_CENTRE_X
    # Treat near-centre as exactly centre so small jitter changes nothing.
    return 0.0 if abs(offset) < CENTRE_DEAD_ZONE else offset


def classify_pillar_position(offset):
    """Human-readable position, for the debug output only."""
    if abs(offset) < POSITION_CENTRE_BAND:
        return "CENTRE"
    return "LEFT" if offset < 0 else "RIGHT"


def lane_offset(left_mm, right_mm):
    """Where we sit across the lane: -1 hard left, +1 hard right, None if unknown.

    Positive means there is more room on the right, i.e. we have drifted left.
    Out-of-range readings are discarded rather than believed - a VL53L0X that
    sees nothing reports about 8190 mm, which would look like a very wide lane.
    """
    if left_mm is None or right_mm is None:
        return None
    if left_mm > LANE_MAX_VALID_MM or right_mm > LANE_MAX_VALID_MM:
        return None
    total = left_mm + right_mm
    if total <= 0:
        return None
    offset = (right_mm - left_mm) / total
    return 0.0 if abs(offset) < LANE_DEAD_ZONE else offset


def boundary_bands(walls, avoid_markers=True):
    """Left and right pixel counts of everything bounding the lane.

    The black track wall always counts. The magenta parking markers count too
    during ordinary driving, because they stand on the track for the whole run
    and a robot that cannot see them will knock them over on every lap.

    While parking they are deliberately excluded. The markers are the target
    then, not an obstacle, and treating them as boundary would steer the robot
    away from the very slot it is trying to enter.
    """
    left, right = walls["left"], walls["right"]
    if avoid_markers:
        left += walls.get("marker_left", 0)
        right += walls.get("marker_right", 0)
    return left, right


def wall_balance(walls, avoid_markers=True):
    """Lane offset from the camera, or None when too little boundary is in view.

    Positive means more boundary on the left, so the robot has drifted left and
    should steer right.
    """
    if not walls:
        return None
    left, right = boundary_bands(walls, avoid_markers)
    total = left + right
    if total < WALL_BAND_MIN_PIXELS:
        return None
    balance = (left - right) / total
    return 0.0 if abs(balance) < LANE_DEAD_ZONE else balance


def compute_lane_steering(left_mm, right_mm, walls=None, avoid_markers=True):
    """Steer back towards the middle of the lane.

    Two independent measurements of the same thing, tried in order of freshness.
    The camera answer is current and looks ahead; the distance sensors are
    accurate but up to a sweep old by the time they are used. Falling back
    rather than averaging keeps the behaviour predictable when one disagrees.
    """
    balance = wall_balance(walls, avoid_markers)
    if balance is not None:
        return (max(-MAX_STEERING_DEG,
                    min(MAX_STEERING_DEG, balance * WALL_BALANCE_GAIN)), balance)

    offset = lane_offset(left_mm, right_mm)
    if offset is None:
        return float(STRAIGHT_STEERING_DEG), 0.0
    return max(-MAX_STEERING_DEG, min(MAX_STEERING_DEG, offset * LANE_GAIN)), offset


def compute_corner_steering(left_mm, right_mm, previous_steering, walls=None):
    """Full lock towards the outside of the corner - the side with more room.

    The camera is asked first for the same reason as lane keeping: mid corner
    the geometry is changing fast, and a stale reading points the wrong way.
    """
    offset = wall_balance(walls)
    if offset is None:
        offset = lane_offset(left_mm, right_mm)
    if offset is None:
        # No side readings mid-corner: hold the turn already in progress rather
        # than straightening up into the wall we are turning away from.
        direction = 1 if previous_steering >= 0 else -1
    else:
        direction = 1 if offset > 0 else -1
    return float(direction * MAX_STEERING_DEG), offset or 0.0


def compute_parking_steering(parking):
    """Aim at the middle of the slot. None when there is no slot in view.

    Unlike a pillar, which we steer to keep beside us, the parking gap is a
    place to drive straight at - so the sign is the plain one: gap to the
    right means steer right.
    """
    if not parking:
        return None
    steering = parking["offset"] * PARKING_GAIN
    return max(-MAX_STEERING_DEG, min(MAX_STEERING_DEG, steering))


def compute_steering(pillar, previous_steering, mode=None,
                     left_mm=None, right_mm=None, parking=None, walls=None):
    """Steering angle for this frame, in degrees.

    Three laws, chosen by mode. Cornering turns towards open space; recentring
    tracks the lane and ignores pillars; otherwise a visible pillar is followed
    to its target position and the lane is followed when there is none.
    """
    lane = 0.0
    if mode in PARKING_MODES:
        # Steer for the slot when we can see it; hold the lane while hunting.
        target = compute_parking_steering(parking)
        if target is None:
            # Hunting for the slot: hold the lane, but do not treat the markers
            # we are looking for as something to steer away from.
            target, lane = compute_lane_steering(left_mm, right_mm, walls,
                                                 avoid_markers=False)
        offset = target_offset = 0.0
    elif mode == MODE_TURN_CORNER:
        target, lane = compute_corner_steering(left_mm, right_mm,
                                               previous_steering, walls)
        offset = target_offset = 0.0
    elif mode == MODE_RECENTER or pillar is None:
        target, lane = compute_lane_steering(left_mm, right_mm, walls)
        offset = target_offset = 0.0
    else:
        colour = pillar["colour"]
        offset = pillar_offset(pillar["x"])
        target_offset = PASS_TARGET[colour]
        target = (offset - target_offset) * STEERING_GAIN

        # Only ever steer around a pillar the way the rules say to pass it.
        # Without this, a pillar already cleared to one side would produce a
        # small correction back towards it - which is how you clip one.
        if PASS_DIRECTION[colour] > 0:
            target = max(0.0, target)
        else:
            target = min(0.0, target)
        target = max(-MAX_STEERING_DEG, min(MAX_STEERING_DEG, target))

    # Rate limit, so the servo sweeps rather than snapping. A single noisy
    # frame can then only move the wheels a few degrees.
    change = target - previous_steering
    change = max(-STEERING_MAX_CHANGE_DEG, min(STEERING_MAX_CHANGE_DEG, change))
    return previous_steering + change, offset, target_offset, lane


def emergency_stop(front_distance, previous_mode):
    """True when there is something too close ahead to keep moving.

    An unknown distance is not an emergency - it means the sensor failed, and
    the speed logic handles that by holding back rather than stopping dead.
    """
    if front_distance is None:
        return False
    # Already stopped? Then require noticeably more room before moving off.
    threshold = STOP_EXIT_MM if previous_mode == MODE_STOP else STOP_ENTER_MM
    return front_distance < threshold


def compute_speed(front_distance, previous_mode, cruise_speed=CRUISE_SPEED):
    """(speed, mode, reason) from the distance ahead."""
    if emergency_stop(front_distance, previous_mode):
        return STOP_SPEED, MODE_STOP, "obstacle at %d mm" % front_distance

    if front_distance is None:
        # No reading at all: assume the worst rather than cruising blind.
        return SLOW_SPEED, MODE_SLOW, "no front reading"

    # Same hysteresis idea as the stop threshold.
    threshold = SLOW_EXIT_MM if previous_mode != MODE_CRUISE else SLOW_ENTER_MM
    if front_distance < threshold:
        return SLOW_SPEED, MODE_SLOW, "obstacle at %d mm" % front_distance
    return cruise_speed, MODE_CRUISE, "path clear"


def mission_speed_limit(mission, mode):
    """Extra ceiling the mission puts on speed, or None for no extra limit.

    Kept apart from compute_speed so the distance-ahead rules stay one thing
    and the mission rules another. The two combine by taking whichever is slower.

    Stopping in the slot is not here: that is a change of behaviour, so the
    state machine owns it. This only sets the pace.
    """
    # Parking is a crawl whether it is the whole mission or the last thing we
    # do in the obstacle round.
    if mode in PARKING_MODES or mission == MISSION_PARKING:
        return MISSION_CRUISE_SPEED[MISSION_PARKING]

    if mission in MISSIONS_WITH_PILLARS and mode in PILLAR_MODES:
        return PILLAR_APPROACH_SPEED

    return None


# ============================================================================
# STATEFUL WRAPPER - the only memory in the file
# ============================================================================

_previous_steering = float(STRAIGHT_STEERING_DEG)
_previous_mode = MODE_CRUISE


def reset():
    """Forget the previous frame. Call before a run, and between tests."""
    global _previous_steering, _previous_mode
    _previous_steering = float(STRAIGHT_STEERING_DEG)
    _previous_mode = MODE_CRUISE


def compute_navigation(vision_data, robot_state, mode=None, mission=None,
                       debug=None):
    """Work out the next movement command.

    Two decisions that never fight each other: steering comes from the pillar,
    the lane or the corner depending on mode; the distance ahead sets the speed.

    `mode` is the state machine's current state from the previous frame. One
    frame of lag is harmless at camera rate, and taking it as an argument keeps
    this module free of any dependency on the state machine.
    """
    global _previous_steering, _previous_mode

    if debug is None:
        debug = DEBUG
    robot_state = robot_state or {}
    front_distance = robot_state.get("front_distance")

    pillar = select_pillar(normalise_pillars(vision_data))
    if mission is not None and mission not in MISSIONS_WITH_PILLARS:
        pillar = None       # not the obstacle round: coloured things are scenery

    parking = vision_data.get("parking") if isinstance(vision_data, dict) else None
    walls = vision_data.get("walls") if isinstance(vision_data, dict) else None
    steering, offset, target_offset, lane = compute_steering(
        pillar, _previous_steering, mode,
        robot_state.get("left_distance"), robot_state.get("right_distance"),
        parking, walls)

    speed, speed_mode, speed_reason = compute_speed(
        front_distance, _previous_mode,
        MISSION_CRUISE_SPEED.get(mission, CRUISE_SPEED))
    limit = mission_speed_limit(mission, mode)
    if limit is not None and limit < speed:
        speed = limit
        speed_reason = "held to %d for this mission" % limit

    _previous_steering, _previous_mode = steering, speed_mode

    if mode in PARKING_MODES:
        reason = ("Parking, slot %+.2f off centre, %s" % (parking["offset"], speed_reason)
                  if parking else "Looking for the parking slot, %s" % speed_reason)
    elif mode == MODE_TURN_CORNER:
        reason = "Cornering towards open space, %s" % speed_reason
    elif mode == MODE_RECENTER:
        reason = "Recentring, lane offset %+.2f, %s" % (lane, speed_reason)
    elif pillar is None:
        reason = "No pillar in view, following the lane, %s" % speed_reason
    else:
        reason = "%s pillar %s of frame, pass %s, %s" % (
            pillar["colour"],
            classify_pillar_position(offset),
            "right" if PASS_DIRECTION[pillar["colour"]] > 0 else "left",
            speed_reason)

    command = {
        "speed": int(speed),
        "steering": int(round(steering)),
        "reason": reason,
        "target_offset": round(target_offset, 3),
        "pillar_offset": round(offset, 3),
        "front_distance": front_distance,
        "lane_offset": round(lane, 3),
        "mode": speed_mode,
        "mission": mission,
    }

    if debug:
        print("Detected: %s" % (pillar["colour"] if pillar else "NONE"))
        print("Position: %s" % classify_pillar_position(offset))
        print("Front Distance: %s" % (
            "%d mm" % front_distance if front_distance is not None else "UNKNOWN"))
        print("Mode: %s" % speed_mode)
        print("Speed: %d" % command["speed"])
        print("Steering: %d" % command["steering"])
        print("Reason: %s." % reason)
        print("-" * 46)

    return command


# ============================================================================
def main():
    """Walk through the decision table so the behaviour is visible."""
    def pillar(colour, x, area=8000, distance=None):
        return {"colour": colour, "x": x, "area": area, "distance": distance}

    cases = (
        ("nothing in view", [], 900),
        ("red centred", [pillar(COLOUR_RED, CAMERA_CENTRE_X)], 700),
        ("red right of centre", [pillar(COLOUR_RED, 415)], 520),
        ("red already cleared to the left", [pillar(COLOUR_RED, 60)], 600),
        ("green centred", [pillar(COLOUR_GREEN, CAMERA_CENTRE_X)], 700),
        ("green left of centre", [pillar(COLOUR_GREEN, 185)], 480),
        ("red near + green far, act on the red",
         [pillar(COLOUR_RED, 410, distance=600), pillar(COLOUR_GREEN, 170, distance=1400)], 700),
        ("green near + red far, act on the green",
         [pillar(COLOUR_GREEN, 170, distance=500), pillar(COLOUR_RED, 410, distance=1500)], 700),
        ("speck of colour, ignored", [pillar(COLOUR_RED, 320, area=90)], 900),
        ("wall closing in", [], 300),
        ("wall too close", [], 120),
        ("front sensor dead", [], None),
    )

    for label, pillars, front in cases:
        reset()
        print("== %s ==" % label)
        # Several frames, so the rate limiter reaches its steady state.
        for _ in range(MAX_STEERING_DEG // STEERING_MAX_CHANGE_DEG + 2):
            compute_navigation({"pillars": pillars}, {"front_distance": front})
        compute_navigation({"pillars": pillars}, {"front_distance": front}, debug=True)


def selftest():
    def pillar(colour, x, area=8000, distance=None):
        return {"colour": colour, "x": x, "area": area, "distance": distance}

    def settle(pillars, front, frames=12):
        """Run the same frame repeatedly, past the rate limiter."""
        reset()
        for _ in range(frames):
            command = compute_navigation({"pillars": pillars},
                                         {"front_distance": front})
        return command

    far, near, blocked = 900, 300, 120
    far_state = {"front_distance": far}

    # --- no pillar ---
    assert settle([], far)["steering"] == STRAIGHT_STEERING_DEG
    reset()
    assert compute_navigation(None, None)["steering"] == STRAIGHT_STEERING_DEG

    # --- pass side is never wrong, anywhere in the frame ---
    for x in (0, 60, 200, CAMERA_CENTRE_X, 415, 500, CAMERA_WIDTH - 1):
        assert settle([pillar(COLOUR_RED, x)], far)["steering"] >= 0, x
        assert settle([pillar(COLOUR_GREEN, x)], far)["steering"] <= 0, x

    # --- a centred pillar demands real correction, a cleared one none ---
    assert settle([pillar(COLOUR_RED, CAMERA_CENTRE_X)], far)["steering"] > 10
    assert settle([pillar(COLOUR_GREEN, CAMERA_CENTRE_X)], far)["steering"] < -10
    assert settle([pillar(COLOUR_RED, 30)], far)["steering"] == 0
    assert settle([pillar(COLOUR_GREEN, CAMERA_WIDTH - 30)], far)["steering"] == 0

    # --- the worse the position, the harder the steer, and never past the limit ---
    assert (settle([pillar(COLOUR_RED, 500)], far)["steering"]
            > settle([pillar(COLOUR_RED, CAMERA_CENTRE_X)], far)["steering"]
            > settle([pillar(COLOUR_RED, 150)], far)["steering"])
    for x in (0, CAMERA_CENTRE_X, CAMERA_WIDTH - 1):
        assert abs(settle([pillar(COLOUR_RED, x)], far)["steering"]) <= MAX_STEERING_DEG
        assert abs(settle([pillar(COLOUR_GREEN, x)], far)["steering"]) <= MAX_STEERING_DEG

    # --- choosing between two visible pillars ---
    red_near = pillar(COLOUR_RED, 410, distance=600)
    green_far = pillar(COLOUR_GREEN, 170, distance=1400)
    assert settle([red_near, green_far], far)["steering"] > 0      # act on the red
    assert settle([green_far, red_near], far)["steering"] > 0      # order must not matter
    green_near = pillar(COLOUR_GREEN, 170, distance=500)
    red_far = pillar(COLOUR_RED, 410, distance=1500)
    assert settle([green_near, red_far], far)["steering"] < 0      # act on the green
    # without distances, the bigger pillar is the nearer one
    assert settle([pillar(COLOUR_RED, 410, area=9000),
                   pillar(COLOUR_GREEN, 170, area=3000)], far)["steering"] > 0

    # --- tiny detections are noise ---
    assert settle([pillar(COLOUR_RED, CAMERA_CENTRE_X, area=MIN_PILLAR_AREA - 1)],
                  far)["steering"] == 0
    assert select_pillar([pillar(COLOUR_RED, 320, area=MIN_PILLAR_AREA)]) is not None

    # --- dead zone: jitter either side of centre gives an identical answer ---
    jitter = int(CAMERA_CENTRE_X * CENTRE_DEAD_ZONE) - 1
    assert (settle([pillar(COLOUR_RED, CAMERA_CENTRE_X + jitter)], far)["steering"]
            == settle([pillar(COLOUR_RED, CAMERA_CENTRE_X - jitter)], far)["steering"])
    assert pillar_offset(CAMERA_CENTRE_X + jitter) == 0.0

    # --- rate limiting: one frame can only move the wheels so far ---
    reset()
    first = compute_navigation({"pillars": [pillar(COLOUR_RED, 500)]},
                               {"front_distance": far})
    assert abs(first["steering"]) <= STEERING_MAX_CHANGE_DEG, first
    second = compute_navigation({"pillars": [pillar(COLOUR_RED, 500)]},
                                {"front_distance": far})
    assert 0 < second["steering"] - first["steering"] <= STEERING_MAX_CHANGE_DEG
    # and it keeps climbing until it reaches the steady state
    assert settle([pillar(COLOUR_RED, 500)], far)["steering"] == MAX_STEERING_DEG

    # --- speed follows the distance ahead ---
    assert settle([], far)["speed"] == CRUISE_SPEED
    assert settle([], near)["speed"] == SLOW_SPEED
    assert settle([], blocked)["speed"] == STOP_SPEED
    assert settle([], None)["speed"] == SLOW_SPEED
    assert settle([], far)["mode"] == MODE_CRUISE
    assert settle([], blocked)["mode"] == MODE_STOP

    # --- hysteresis: between the two thresholds, the previous mode wins ---
    between = (SLOW_ENTER_MM + SLOW_EXIT_MM) // 2
    reset()
    compute_navigation({}, {"front_distance": far})                 # cruising
    assert compute_navigation({}, {"front_distance": between})["mode"] == MODE_CRUISE
    reset()
    compute_navigation({}, {"front_distance": near})                # slowed
    assert compute_navigation({}, {"front_distance": between})["mode"] == MODE_SLOW
    # same again for the stop threshold
    stop_between = (STOP_ENTER_MM + STOP_EXIT_MM) // 2
    reset()
    compute_navigation({}, {"front_distance": blocked})             # stopped
    assert compute_navigation({}, {"front_distance": stop_between})["mode"] == MODE_STOP
    reset()
    assert compute_navigation({}, {"front_distance": stop_between})["mode"] == MODE_SLOW

    # --- a pillar never overrides an emergency stop ---
    stopped = settle([pillar(COLOUR_RED, CAMERA_CENTRE_X)], blocked)
    assert stopped["speed"] == STOP_SPEED and stopped["steering"] > 0

    # --- the returned dictionary is complete and UART-ready ---
    command = settle([pillar(COLOUR_RED, 415)], 520)
    assert set(command) == {"speed", "steering", "reason", "target_offset",
                            "pillar_offset", "front_distance", "mode",
                            "lane_offset", "mission"}, sorted(command)
    assert isinstance(command["speed"], int) and isinstance(command["steering"], int)
    assert 0 <= command["speed"] <= 100
    assert command["target_offset"] == -PASS_TARGET_OFFSET
    assert command["front_distance"] == 520 and command["mode"] == MODE_CRUISE

    # --- the older single-pillar vision format still works ---
    reset()
    legacy = settle(None, far)
    assert legacy["steering"] == 0
    reset()
    for _ in range(12):
        old = compute_navigation(
            {"largest_colour": COLOUR_RED, "center_x": 415, "area": 8200},
            {"front_distance": far})
    assert old["steering"] > 0, old

    # --- lane keeping ---
    assert lane_offset(500, 500) == 0.0              # dead centre
    assert lane_offset(400, 600) > 0                 # more room right -> drifted left
    assert lane_offset(600, 400) < 0
    assert lane_offset(None, 500) is None            # one sensor dead
    assert lane_offset(500, None) is None
    assert lane_offset(0, 0) is None                 # nonsense, not a divide by zero
    assert lane_offset(500, 9000) is None            # 8190 = "sees nothing", not a lane
    # small drift is inside the dead zone and must not twitch the wheels
    assert lane_offset(495, 505) == 0.0

    # steering follows: drifted left -> steer right, and never past the limit
    assert compute_lane_steering(400, 600)[0] > 0
    assert compute_lane_steering(600, 400)[0] < 0
    assert compute_lane_steering(None, None)[0] == STRAIGHT_STEERING_DEG
    for pair in ((1, 1999), (1999, 1), (500, 500)):
        assert abs(compute_lane_steering(*pair)[0]) <= MAX_STEERING_DEG

    # the lane law applies whenever no pillar is in view
    lane_state = {"front_distance": far, "left_distance": 400, "right_distance": 600}
    reset()
    for _ in range(12):
        drifted = compute_navigation({"pillars": []}, lane_state)
    assert drifted["steering"] > 0, drifted
    assert drifted["lane_offset"] > 0

    # --- recentring ignores pillars and tracks the lane instead ---
    reset()
    for _ in range(12):
        recentring = compute_navigation(
            {"pillars": [pillar(COLOUR_GREEN, CAMERA_CENTRE_X)]}, lane_state,
            mode=MODE_RECENTER)
    assert recentring["steering"] > 0, recentring      # lane says right...
    reset()
    for _ in range(12):
        following = compute_navigation(
            {"pillars": [pillar(COLOUR_GREEN, CAMERA_CENTRE_X)]}, lane_state)
    assert following["steering"] < 0, following        # ...but the pillar says left

    # --- cornering turns towards open space, at full lock ---
    assert compute_corner_steering(300, 1500, 0)[0] == MAX_STEERING_DEG
    assert compute_corner_steering(1500, 300, 0)[0] == -MAX_STEERING_DEG
    # no side readings mid-turn: hold the turn, do not straighten into the wall
    assert compute_corner_steering(None, None, -20)[0] == -MAX_STEERING_DEG
    assert compute_corner_steering(None, None, 20)[0] == MAX_STEERING_DEG

    reset()
    corner_state = {"front_distance": 500, "left_distance": 300, "right_distance": 1500}
    for _ in range(12):
        cornering = compute_navigation({"pillars": []}, corner_state,
                                       mode=MODE_TURN_CORNER)
    assert cornering["steering"] == MAX_STEERING_DEG, cornering

    # a pillar in view must not divert a corner
    reset()
    for _ in range(12):
        cornering = compute_navigation(
            {"pillars": [pillar(COLOUR_GREEN, CAMERA_CENTRE_X)]}, corner_state,
            mode=MODE_TURN_CORNER)
    assert cornering["steering"] == MAX_STEERING_DEG, cornering

    # --- the old two-argument call still behaves exactly as it did ---
    reset()
    for _ in range(12):
        legacy_call = compute_navigation({"pillars": [pillar(COLOUR_RED, 415)]}, far_state)
    assert legacy_call["steering"] > 0 and legacy_call["lane_offset"] == 0.0

    # --- lane position from the camera ---
    hug_left = {"left": 900, "right": 100, "balance": 0.8}
    hug_right = {"left": 100, "right": 900, "balance": -0.8}
    centred_walls = {"left": 500, "right": 500, "balance": 0.0}
    no_wall = {"left": 5, "right": 5, "balance": 0.0}

    assert wall_balance(hug_left) > 0            # drifted left
    assert wall_balance(hug_right) < 0
    assert wall_balance(centred_walls) == 0.0
    assert wall_balance(no_wall) is None         # too little wall to trust
    assert wall_balance(None) is None
    # jitter inside the dead zone must not move the wheels
    assert wall_balance({"left": 510, "right": 490, "balance": 0.02}) == 0.0

    # drifted left means steer right, and the reverse
    assert compute_lane_steering(None, None, hug_left)[0] > 0
    assert compute_lane_steering(None, None, hug_right)[0] < 0
    for walls in (hug_left, hug_right):
        assert abs(compute_lane_steering(None, None, walls)[0]) <= MAX_STEERING_DEG

    # the camera is preferred over the distance sensors when both are available
    disagreeing = compute_lane_steering(900, 100, hug_left)[0]   # ToF says left
    assert disagreeing > 0, disagreeing                          # camera wins

    # and the sensors are used when there is no wall in view
    assert compute_lane_steering(400, 600, no_wall)[0] > 0
    assert compute_lane_steering(None, None, None)[0] == STRAIGHT_STEERING_DEG

    # end to end, through the whole engine
    reset()
    for _ in range(14):
        camera_lane = compute_navigation(
            {"pillars": [], "walls": hug_left}, {"front_distance": far})
    assert camera_lane["steering"] > 0, camera_lane

    # --- the parking markers are boundary while driving, target while parking ---
    marker_on_left = {"left": 0, "right": 0, "balance": 0.0,
                      "marker_left": 700, "marker_right": 0}

    # driving past them: they push the robot away, to the right
    assert wall_balance(marker_on_left) > 0.9
    assert compute_lane_steering(None, None, marker_on_left)[0] > 0

    # ...and a marker on the right pushes the other way
    marker_on_right = {"left": 0, "right": 0, "balance": 0.0,
                       "marker_left": 0, "marker_right": 700}
    assert compute_lane_steering(None, None, marker_on_right)[0] < 0

    # while parking they are ignored, so we are not pushed off the slot
    assert wall_balance(marker_on_left, avoid_markers=False) is None
    assert compute_lane_steering(None, None, marker_on_left,
                                 avoid_markers=False)[0] == STRAIGHT_STEERING_DEG

    # wall and markers on opposite sides add up rather than cancelling wrongly
    both = {"left": 400, "right": 0, "balance": 1.0,
            "marker_left": 0, "marker_right": 400}
    assert abs(wall_balance(both)) < LANE_DEAD_ZONE, wall_balance(both)

    # end to end: a marker ahead on the left steers the robot right, every mode
    for parking_mode in (None, MODE_RECENTER):
        reset()
        for _ in range(14):
            avoided = compute_navigation({"pillars": [], "walls": marker_on_left},
                                         {"front_distance": far}, parking_mode)
        assert avoided["steering"] > 0, (parking_mode, avoided)

    # --- missions ---
    red_centred = [pillar(COLOUR_RED, CAMERA_CENTRE_X)]
    lane_state = {"front_distance": far, "left_distance": 400, "right_distance": 600}

    def settle_mission(pillars, state, mission, mode=None, frames=14):
        reset()
        for _ in range(frames):
            command = compute_navigation({"pillars": pillars}, state, mode, mission)
        return command

    # OPEN: pillars are scenery. Follow the lane and run harder.
    open_run = settle_mission(red_centred, lane_state, MISSION_OPEN)
    assert open_run["steering"] > 0, open_run          # lane says right...
    obstacle_run = settle_mission(red_centred, lane_state, MISSION_OBSTACLE)
    assert obstacle_run["steering"] > 0                # ...and so does the red pillar
    green_run = settle_mission([pillar(COLOUR_GREEN, CAMERA_CENTRE_X)],
                               lane_state, MISSION_OBSTACLE)
    assert green_run["steering"] < 0                   # green overrides the lane
    open_green = settle_mission([pillar(COLOUR_GREEN, CAMERA_CENTRE_X)],
                                lane_state, MISSION_OPEN)
    assert open_green["steering"] > 0, open_green      # ...but not in the open round

    assert open_run["speed"] == MISSION_CRUISE_SPEED[MISSION_OPEN]
    assert open_run["speed"] > obstacle_run["speed"]
    assert open_run["mission"] == MISSION_OPEN

    # OBSTACLE: ease off while working a pillar, full speed otherwise
    approaching = settle_mission(red_centred, {"front_distance": far},
                                 MISSION_OBSTACLE, MODE_APPROACH_PILLAR)
    passing = settle_mission(red_centred, {"front_distance": far},
                             MISSION_OBSTACLE, MODE_PASS_PILLAR)
    assert approaching["speed"] == PILLAR_APPROACH_SPEED, approaching
    assert passing["speed"] == PILLAR_APPROACH_SPEED
    assert approaching["speed"] < obstacle_run["speed"]
    # the open round has no pillar phases to slow down for
    assert settle_mission(red_centred, {"front_distance": far},
                          MISSION_OPEN, MODE_APPROACH_PILLAR)["speed"] \
        == MISSION_CRUISE_SPEED[MISSION_OPEN]

    # PARKING: crawl, then stop once the rear sensor says we are in
    parking = settle_mission([], {"front_distance": far, "rear_distance": 800},
                             MISSION_PARKING)
    assert parking["speed"] == MISSION_CRUISE_SPEED[MISSION_PARKING], parking
    # navigation only sets the pace while parking - deciding we have arrived is
    # a change of behaviour, so the state machine does it
    for parking_mode in PARKING_MODES:
        crawl = settle_mission([], {"front_distance": far}, MISSION_PARKING,
                               parking_mode)
        assert crawl["speed"] == MISSION_CRUISE_SPEED[MISSION_PARKING], crawl
    assert mission_speed_limit(MISSION_OBSTACLE, MODE_ENTER_PARKING) \
        == MISSION_CRUISE_SPEED[MISSION_PARKING]
    assert mission_speed_limit(MISSION_OPEN, None) is None

    # --- parking steers at the slot, and crawls, in any mission ---
    def park_frame(slot, state, mode, frames=14):
        reset()
        for _ in range(frames):
            command = compute_navigation({"pillars": [], "parking": slot},
                                         state, mode, MISSION_OBSTACLE)
        return command

    slot_right = {"offset": 0.6, "markers": 2}
    slot_left = {"offset": -0.6, "markers": 2}
    plain = {"front_distance": far}
    assert park_frame(slot_right, plain, MODE_ALIGN_PARKING)["steering"] > 0
    assert park_frame(slot_left, plain, MODE_ALIGN_PARKING)["steering"] < 0
    assert park_frame({"offset": 0.0, "markers": 2}, plain,
                      MODE_ALIGN_PARKING)["steering"] == 0
    # parking is a crawl even in the middle of the obstacle round
    assert park_frame(slot_right, plain,
                      MODE_ALIGN_PARKING)["speed"] == MISSION_CRUISE_SPEED[MISSION_PARKING]
    # no slot in view yet: hold the lane rather than steering at nothing
    hunting = park_frame(None, {"front_distance": far, "left_distance": 400,
                                "right_distance": 600}, MODE_SEARCH_PARKING)
    assert hunting["steering"] > 0, hunting
    # and a pillar must not divert us once we are parking
    assert park_frame(slot_right, plain, MODE_ALIGN_PARKING)["steering"] > 0
    assert abs(compute_parking_steering({"offset": 5.0})) <= MAX_STEERING_DEG
    assert compute_parking_steering(None) is None

    # a mission limit can only ever slow us down, never speed us up
    for mission in (MISSION_OPEN, MISSION_OBSTACLE, MISSION_PARKING):
        blocked_run = settle_mission([], {"front_distance": blocked}, mission)
        assert blocked_run["speed"] == STOP_SPEED, mission

    # no mission given: exactly the behaviour this module had before
    assert settle_mission(red_centred, {"front_distance": far},
                          None)["speed"] == CRUISE_SPEED

    print("selftest ok  red centred -> %+d deg   green centred -> %+d deg"
          % (settle([pillar(COLOUR_RED, CAMERA_CENTRE_X)], far)["steering"],
             settle([pillar(COLOUR_GREEN, CAMERA_CENTRE_X)], far)["steering"]))


if __name__ == "__main__":
    selftest() if "--selftest" in sys.argv else main()
