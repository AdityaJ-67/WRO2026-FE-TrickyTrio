# Tricky Trio Code Explained

A line by line walkthrough of every file in the robot, written for someone who has not
seen the code before.

## How to read this document

The robot has two computers, so this document has two halves.

**PI** is the Raspberry Pi 3. It has a camera, runs Linux, and does the thinking. It
answers questions like "is there a red sign ahead" and "how hard should I turn".

**PICO** is the Raspberry Pi Pico 2 W. It is a microcontroller, which means it has no
operating system and does exactly one thing at a time, very reliably. It answers
questions like "put the servo at 20 degrees" and "how many times has the wheel turned".

They talk to each other over three wires.

## The one idea that explains the whole design

Linux can pause. If the Pi decides to save a file, or the Wi-Fi chip interrupts it, your
program stops for a few milliseconds and then carries on. You never notice this when
you are browsing the internet.

A servo notices. A servo needs a pulse every 20 milliseconds, forever. Miss one and it
twitches.

So the rule is: **anything with a deadline runs on the Pico, anything that needs to
think runs on the Pi.** Almost every design decision in this code follows from that one
sentence.

## Vocabulary you will need

**Module.** One file of Python code. `servo.py` is a module.

**Function.** A named block of code you can run. `def move(speed, steering):` defines a
function called `move` that takes two pieces of information.

**Argument.** The information you hand a function. In `move(45, 20)`, the arguments are
45 and 20.

**Return.** What a function hands back when it finishes.

**Constant.** A value with a name, written in CAPITALS, that does not change while the
program runs. `MAX_STEERING_DEG = 30`.

**Dictionary.** A lookup table. `{"speed": 45, "steering": 20}` lets you ask for
`command["speed"]` and get 45.

**Tuple.** A list that cannot be changed after it is made, written with round brackets.

**PWM.** Pulse Width Modulation. A way of getting an on/off pin to act like a dial, by
switching it on and off very fast and varying how long it stays on.

**I2C.** A way for several chips to share two wires. Each chip has an address so they
know who is being spoken to.

**Self test.** A function inside a file that checks the file works. Every module here
has one, and they all run on a laptop with no robot attached.

# PI

Everything in this half runs on the Raspberry Pi 3, in normal Python 3.

Read the files in this order. Each one builds on the last.

## PI / mission_manager.py

**Job: say which of the three competition rounds we are running.**

This is the smallest file in the project, 44 lines, and it is a good place to start
because it shows the shape every other file follows.

```python
class Mission(Enum):
    OPEN_CHALLENGE = "OPEN_CHALLENGE"
    OBSTACLE_CHALLENGE = "OBSTACLE_CHALLENGE"
    PARKING = "PARKING"
```

An **Enum** is a fixed list of allowed values. Without it we would pass the text
`"OPEN_CHALLENGE"` around, and a typo like `"OPEN_CHALENGE"` would not be noticed until
the robot behaved strangely on the mat. With an Enum, `Mission.OPEN_CHALENGE` fails
instantly with an error, on the laptop, the moment you run it.

```python
MISSION = Mission.OBSTACLE_CHALLENGE
```

The one line you change before a run. It sits alone, in capitals, at the top, so nobody
has to hunt for it.

```python
def current_mission():
    return {"mission": MISSION.value, "reason": "Selected manually in config"}
```

Two things worth noticing.

`MISSION.value` gives the plain text `"OBSTACLE_CHALLENGE"` rather than the Enum object,
because the rest of the program works with text.

`"reason"` looks pointless right now, since the answer is always the same. It exists
because later this file will detect the mission automatically, and then the reason will
be something like "magenta markers visible, assuming parking". Everything that calls
this function already prints the reason, so when that day comes nothing else has to
change.

**Why this is a whole file for four lines of logic.** Because `main.py` asks it every
single frame rather than once at startup. That means swapping the constant for real
detection later touches only this file.

## PI / camera_vision/vision_test.py

**Job: look at a camera frame and say what is in it.**

330 lines. This is where the robot gets its eyes.

### The problem colour detection has to solve

A camera gives you red, green and blue values for every pixel. That sounds perfect for
finding a red sign, until you realise that a red sign in shadow has a *lower* red value
than a white wall in bright sun. Brightness contaminates everything.

**HSV** solves this. It describes a colour as three different numbers:

- **Hue**, what colour it is, as an angle from 0 to 179 in OpenCV
- **Saturation**, how strong the colour is, 0 to 255
- **Value**, how bright it is, 0 to 255

Now "red" is a hue range, and brightness lives in a separate number we can be relaxed
about. That is why the first thing the code does is convert.

### The colour ranges

```python
COLOUR_RANGES = {
    "RED": [((0, 90, 40), (10, 255, 255)), ((172, 90, 40), (180, 255, 255))],
    "GREEN": [((35, 70, 35), (90, 255, 255))],
}
```

Each entry is a lowest and highest HSV value. Anything between them counts.

**Red needs two ranges** because hue is a circle and red sits at the join. Hue 0 and hue
179 are both red, the way 23:59 and 00:01 are both nearly midnight. So we look for
0 to 10 and 172 to 180 and combine them.

**Why the saturation and value floors are so low.** Our first version used 110 and 70.
It worked perfectly with one sign and kept losing the second one. Two signs on a mat are
never lit equally, and the one further from the lights fell below the floor. It looked
random because it depended on where the robot was standing. Lowering the floors fixed it.

**Why red now starts at 172 and not 165.** Magenta, the parking marker colour, sits just
below red on the hue circle. With red starting at 165 the robot saw the parking lot as
one enormous traffic sign and tried to pass it on the right. Moving the start to 172
costs nothing, because red's main band is 0 to 10.

### Making a mask

```python
def colour_mask(hsv, ranges):
    mask = cv2.inRange(hsv, ...)
    for low, high in ranges[1:]:
        mask |= cv2.inRange(hsv, ...)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)
```

`cv2.inRange` returns a **mask**: a black and white image where white means "this pixel
was inside the range". The `|=` merges red's two ranges into one mask.

The two morphology steps clean it up.

**Open** removes tiny white specks, the way you would brush dust off a photo. Camera
noise produces single stray pixels that pass the colour test.

**Close** fills small black holes inside white areas. A glossy sign has a bright
reflection on it that is not red, punching a hole in the middle of the shape.

Doing open first then close is deliberate: remove the dust before you fill the holes,
or you fill the dust in too.

### Finding the sign

```python
def find_block(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_area = None, MIN_AREA
    for contour in contours:
        area = cv2.contourArea(contour)
        if area <= best_area:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if not MIN_RATIO <= w / h <= MAX_RATIO:
            continue
        if area / (w * h) < MIN_FILL:
            continue
        best, best_area = (x, y, w, h), area
    return best
```

A **contour** is the outline of a connected white blob. This function picks the best one.

The obvious approach is "take the biggest blob". That is wrong, and it took us a while
to see why. The biggest red blob in the room might be a jacket in the audience, or a
long stripe of glare across the mat.

So a blob has to pass three tests:

**Area.** Bigger than `MIN_AREA`. Notice the trick: `best_area` starts at `MIN_AREA`, so
the single comparison `area <= best_area` does two jobs at once. It rejects blobs that
are too small *and* blobs that are worse than the best one found so far.

**Aspect ratio.** Width divided by height must be between 0.15 and 2.00. Signs are
taller than wide. A long horizontal stripe of glare has a ratio around 12 and fails.

**Fill ratio.** The contour's area divided by its bounding box area. A solid sign fills
most of its box, giving something above 0.8. Scattered reflections might have a large
total area but be spread out, giving 0.2. This is the test that rejects a person.

The self test deliberately puts a wide red stripe with the *largest area in the frame*
into a test image and checks the detector skips it in favour of the real sign.

**Why one pass and not sorting.** An earlier version sorted all contours by size and
walked the list. Sorting is unnecessary work when you only want the best one, and it
computed each area twice. This version does one pass and computes each area once.

### Working out how far away something is

```python
focal_px = (width / 2) / TAN_HALF_HFOV
distance = real_height_cm * focal_px / max(h, 1)
```

This is the whole reason we can get range from one camera with no second lens.

A sign is a known 10 cm tall. The further away it is, the fewer pixels tall it appears,
and the relationship is a simple division. Double the distance, half the pixel height.

`focal_px` converts between the two, and it depends on the camera's field of view, which
is why `CAMERA_HFOV_DEG` has to be calibrated for your camera.

`max(h, 1)` prevents dividing by zero if the height ever came out as 0.

`TAN_HALF_HFOV` is calculated once at the top of the file rather than every frame,
because it never changes.

### The region of interest

```python
roi_y = int(height * ROI_TOP)
small = cv2.resize(frame[roi_y:], (PROC_WIDTH, proc_height), ...)
```

Two speedups in two lines.

`frame[roi_y:]` throws away the top 35 percent of the image. Signs stand on the mat, so
the top of the frame contains only ceiling and lights. Ignoring it is free and removes a
whole category of false detection.

`cv2.resize` then shrinks what remains to 320 pixels wide. Detection does not need
detail; it needs to know roughly where a coloured rectangle is. Working at 320 instead
of 640 means roughly four times fewer pixels to examine.

Drawing still happens at full resolution, so what you see on screen is sharp.

The measured result is 0.32 milliseconds per frame, which means the camera's frame rate
limits the robot, not the code. That is the right way round.

### Where things are, left to right

```python
def direction(cx, width):
    offset = (cx - width / 2) / (width / 2)
    if offset < -CENTRE_DEADBAND:
        return offset, "LEFT"
    ...
```

`offset` converts a pixel position into a number from -1 to +1, where 0 is dead centre.
Everything downstream uses this instead of pixels, so the logic does not care what
resolution the camera is running at.

### The parking markers

```python
def find_markers(mask):
    ...
    found.sort(key=lambda box: box[2] * box[3], reverse=True)
    return sorted(found[:2], key=lambda box: box[0])
```

Two sorts, doing different jobs. The first picks the two biggest magenta blobs, on the
assumption that anything else magenta in the room is smaller. The second puts those two
back into left to right order, so we know which is which.

```python
gap_x = (left[0] + left[2] + right[0]) // 2
```

`left[0] + left[2]` is the right hand edge of the left marker. `right[0]` is the left
hand edge of the right marker. Halfway between them is the middle of the **gap**, which
is the slot itself, not either marker. That is where we want to drive.

## PI / navigation_engine.py

**Job: decide how fast to go and which way to steer.**

741 lines, and the most important file to understand. It answers *how to drive*. It does
not decide *what we are doing*; that is the next file.

### Everything is a pure function

Almost every function here is **pure**, which means it only looks at what you hand it,
and always gives the same answer for the same input. It reads no hidden state.

That sounds academic. It is why the self test can check dozens of situations in
milliseconds with no robot: you just hand the function a situation and check the answer.

### The steering idea that removes nine cases

The rule is red on the right, green on the left. The obvious way to code that is a set
of cases: red on the left, red centred, red on the right, then three more for green,
then "no sign". Nine branches, nine chances to make a mistake.

Instead:

```python
PASS_TARGET = {COLOUR_RED: -0.5, COLOUR_GREEN: 0.5}
target = (offset - PASS_TARGET[colour]) * STEERING_GAIN
```

Each colour gets a **target position in the frame**. Passing a red sign on its right
means the sign has to end up on our left, so the target for red is -0.5. Then we steer
by however far the sign is from where it should be.

Work through it:

| Sign at | offset | offset minus target | Steering |
|---|---|---|---|
| Far left | -0.91 | -0.41 | negative, clamped to 0 |
| Centred | 0.00 | +0.50 | firm right |
| Far right | +0.56 | +1.06 | full lock right |

A centred sign produces a strong correction and a sign far to one side produces none.
That is the correct behaviour, and nobody had to write it down as a rule.

### The clamp that stops us clipping signs

```python
if PASS_DIRECTION[colour] > 0:
    target = max(0.0, target)
else:
    target = min(0.0, target)
```

A red sign may only ever produce right steering. Without this, a sign already safely
past on our left would produce a small correction back *towards* it. That is exactly
how you clip one.

### Lane following without knowing the lane width

```python
offset = (right_mm - left_mm) / (right_mm + left_mm)
```

This is my favourite line in the project. The two angled sensors report distances. If
there is more room on the right, we have drifted left, so we steer right.

Dividing by the total is what makes it work in any lane width. In a 1 metre lane and a
2 metre lane, being one third of the way across gives the same answer.

```python
if left_mm > LANE_MAX_VALID_MM or right_mm > LANE_MAX_VALID_MM:
    return None
```

A VL53L0X reports about 8190 mm when it sees nothing at all. Feed that into the formula
and it looks like an enormously wide lane, and the robot steers confidently in the wrong
direction. Anything over 2000 mm is treated as no reading rather than believed.

### Not turning too fast

```python
change = target - previous_steering
change = max(-STEERING_MAX_CHANGE_DEG, min(STEERING_MAX_CHANGE_DEG, change))
return previous_steering + change, ...
```

The **rate limit**. The steering angle may change by at most 6 degrees per frame, so
reaching full lock takes five frames, about 170 milliseconds at 30 frames per second.

One bad detection can now only move the wheels 6 degrees instead of slamming them over.
The servo also sweeps rather than snapping, which is gentler on the linkage.

This is the one number to change if the robot feels sluggish or twitchy on the mat.

### Hysteresis, or why the robot stopped surging

```python
SLOW_ENTER_MM = 400
SLOW_EXIT_MM = 500
```

Two thresholds for one decision. Drop below 400 to slow down, but you must get back
above 500 to speed up again.

With a single threshold at 400 and a sensor that wobbles by a few millimetres, readings
of 399, 401, 398, 402 flip the mode four times in four frames. The robot audibly surged
and slowed several times a second. Two thresholds mean a reading near the boundary
cannot cause a change.

The same idea is used for the emergency stop, at 150 and 220.

### Choosing between two signs

```python
if all(p.get("distance") is not None for p in candidates):
    return min(candidates, key=lambda p: p["distance"])
return max(candidates, key=lambda p: p["area"])
```

When a red and a green are both visible, the near one is about to be hit, so that is the
one we act on. The far one gets handled later, once it becomes the near one.

If distances are known we use them. If not, we fall back to apparent size, since a
nearer sign of the same real size looks bigger. The fallback exists so the function
still works if the vision module is ever changed to not measure distance.

### Three steering laws, chosen by mode

```python
if mode in PARKING_MODES:      ... aim at the parking gap
elif mode == MODE_TURN_CORNER: ... full lock towards open space
elif mode == MODE_RECENTER:    ... ignore signs, find the lane centre
elif pillar:                   ... follow the sign
else:                          ... follow the lane
```

The same function handles every situation, and `mode` picks which rule applies. `mode`
comes from the state machine, which is the next file.

Notice that recentring deliberately ignores signs. That is the whole point of it: after
squeezing past one obstacle, get back to the middle of the lane before worrying about
the next.

### Cornering when the sensors fail

```python
if offset is None:
    direction = 1 if previous_steering >= 0 else -1
```

If both side sensors fail in the middle of a corner, we keep turning the way we were
already turning. The alternative, straightening up, would drive into the wall we were
turning away from.

## PI / state_machine.py

**Job: decide what the robot is currently doing.**

966 lines, the largest file. Navigation says *how to drive*. This says *what we are
doing*, which is a different question.

Why separate them? Because "there is a wall 500 mm ahead" means different things
depending on the situation. On a straight it means a corner is coming. Halfway through a
corner it is just the wall you are turning away from. Without a state, every module ends
up guessing at that context and they disagree with each other.

### States and events

A **state** is what the robot is doing. There are twelve, including `FOLLOW_COURSE`,
`APPROACH_PILLAR`, `TURN_CORNER` and `FINISHED`.

An **event** is something the robot noticed. There are seventeen, including
`CORNER_DETECTED`, `RED_PILLAR` and `PARKED`.

### The split that makes this file readable

```python
def detect_events(context):
    if front < CORNER_TRIGGER_MM:
        events.append(Event.CORNER_DETECTED)
```

`detect_events` is the **only** function that knows about millimetres and degrees. It
turns raw numbers into named events.

```python
State.FOLLOW_COURSE: (
    (Event.RUN_COMPLETE,      State.SEARCH_PARKING),
    (Event.RECOVERY_REQUIRED, State.RECOVERY),
    (Event.CORNER_DETECTED,   State.TURN_CORNER),
    (Event.RED_PILLAR,        State.APPROACH_PILLAR),
    (Event.GREEN_PILLAR,      State.APPROACH_PILLAR),
),
```

The `TRANSITIONS` table is the **only** place that knows about behaviour, and it
contains no numbers at all.

That separation is the point. When we improve how a corner is detected, we change
`detect_events`. The table never learns that anything changed.

**Row order is priority.** Reading down that list: finishing beats everything, then
trouble, then a corner, then a sign. "A wall beats a sign" is a line you can point at,
not something that emerges from the order somebody happened to write their if statements
in.

### The whole engine is five lines

```python
for event, next_state in TRANSITIONS[self.state]:
    if event not in self.events:
        continue
    target = allowed_state(next_state, self.mission)
    if target is None:
        continue
    self._change_to(target, event, context)
    break
```

Look up the current state's row, find the first event that actually happened, go there,
stop looking.

### Events fire whether they matter or not

`PATH_CLEAR` is reported every single frame the way ahead is open, even while cruising
along happily. `FOLLOW_COURSE` simply does not list it, so it is ignored.

This is deliberate and it is what makes the design extensible. A new state can consume an
event that already exists without touching the detection code at all. There is a test
that proves it: the robot reports seeing a red sign in the Open Challenge and correctly
does nothing about it.

### Missions gate which states exist

```python
MISSION_STATES = {
    "OPEN_CHALLENGE": CORE_STATES | frozenset((FOLLOW_COURSE, TURN_CORNER)),
    ...
}
MISSION_SUBSTITUTE = {
    "OPEN_CHALLENGE": {
        State.APPROACH_PILLAR: None,
        State.SEARCH_PARKING: State.FINISHED,
    },
}
```

In the Open Challenge there are no signs to dodge, so `APPROACH_PILLAR` does not exist
and the substitute is `None`, meaning "skip that transition entirely".

The `SEARCH_PARKING: FINISHED` line is doing something important. Without it, an open
challenge robot would finish three laps, be told to go and park, find no parking state,
and drive forever. A test checks that every transition in every mission leads somewhere.

### One timeout mechanism instead of four

```python
STATE_TIMEOUT_S = {
    State.INITIALISE: 1.0,
    State.APPROACH_PILLAR: 6.0,
    State.TURN_CORNER: 4.0,
}
```

A single `STATE_TIMEOUT` event plus a table. A state that is not in the table never
times out, which is how `FOLLOW_COURSE` can run all day and `FINISHED` is permanent,
without either needing a special case.

### Restraining navigation, never replacing it

```python
STATE_SPEED_CAP = {State.TURN_CORNER: 35, State.RECOVERY: 30, State.FINISHED: 0, ...}
STATE_SPEED_FLOOR = {State.TURN_CORNER: 20, State.ENTER_PARKING: 15, ...}
```

The state machine may only **restrain** navigation's request. `None` means take whatever
navigation asked for. A cap can only slow you down.

The **floor** exists because of a real bug. Turning towards a wall, the front sensor
keeps closing, so navigation's emergency stop fired mid corner. But a stopped robot
stops turning, so the corner never completed, so the state timed out into recovery, and
recovery could not clear either because the wall was still there. Four correct
behaviours chaining into a dead end. The floor makes the robot creep through instead of
freezing.

### The bug that only appeared at time zero

```python
if self._stuck_since is None:
    self._stuck_since = now
```

This was originally `self._stuck_since = self._stuck_since or now`. In Python, `or`
treats `0` as "nothing", so a timestamp of exactly `0.0` was read as "not set" and the
timer reset itself every frame. Stall detection never fired in the first seconds of a
run. Worth knowing because `or` is such a natural thing to write.

## PI / main.py

**Job: wire the other files together. Nothing else.**

406 lines, and it contains no decisions at all. It captures, converts, passes along and
sends.

### The loop, in order

```python
captured, frame = camera.read()          # 1
seen, _, _ = vision_test.detect(frame)   # 2
robot_state = read_state(link, ...)      # 3
selected = mission_manager.current_mission()  # 4
navigation = navigation_engine.compute_navigation(...)   # 5
behaviour = machine.update(vision, robot_state, navigation)  # 6
link.write(...)                          # 7
```

Seven steps, seven lines. Step 7 sends `behaviour`, never `navigation`, so there is
exactly one place in the entire system where a command can reach the wheels.

### Converting between modules

```python
"distance": info["distance"] * CM_TO_MM
```

Vision works in centimetres, navigation works in millimetres. Rather than let both
modules guess, main converts explicitly at the boundary. Unit mismatches are a classic
way to lose a robot.

### Draining the serial buffer

```python
while link.in_waiting:
    parsed = parse_state(link.readline().decode(errors="ignore"))
    if parsed:
        state = parsed
```

Read every waiting line and keep the **newest**. If one camera frame took a long time,
several sensor updates have piled up, and acting on the oldest would mean steering by
stale distances.

`errors="ignore"` stops a single corrupted byte from crashing the program.

### Always stopping

```python
finally:
    if link:
        for _ in range(STOP_REPEATS):
            link.write(STOP_COMMAND.encode())
```

`finally` runs no matter how the loop ended: normal exit, Ctrl-C, or a crash. Without
it, a crash in the Pi leaves the Pico driving at the last speed it was told, forever.

Sent three times because a single message could be lost.

### Keeping the frame width honest

```python
if navigation_engine.CAMERA_WIDTH != width:
    navigation_engine.CAMERA_WIDTH = width
    navigation_engine.CAMERA_CENTRE_X = width // 2
```

Every sign position is measured against the frame centre. If the camera returns
something other than the expected width, the centre is wrong and every steering decision
is quietly biased. This checks reality instead of trusting a constant.

## PI / uart_test_pi.py

**Job: prove the Pi can talk to the Pico, and nothing else.**

75 lines. A bench tool, not part of the competition program. It sends four commands on a
loop so you can watch them arrive on the Pico.

```python
def parse(line):
    values = [int(v) for v in line.strip().split(",")]
    return values[0], values[1], values[2:]
```

Note `values[2:]`, which collects any extra fields. Today we send `speed,steering`. If
we later send `speed,steering,state,flags`, this still works and the extras land in the
spare list. The self test checks exactly that, which is how we know the message format
can grow without breaking either board.

# PICO

Everything in this half runs on the Raspberry Pi Pico 2 W, in MicroPython.

**MicroPython** is Python cut down to fit on a microcontroller. Most things work the
same. The differences that matter here:

- There is a `machine` module for talking to pins, which does not exist on a laptop
- There is no operating system, so your program is the only thing running
- Memory is small, so allocating memory inside an interrupt is forbidden
- `time.ticks_ms()` replaces the usual timing functions

### The import trick used in every Pico file

```python
try:
    from machine import PWM, Pin, UART
except ImportError:
    PWM = Pin = UART = None
```

On the Pico this import works. On a laptop it fails, and instead of crashing we set the
names to `None`.

Why bother? Because it means the *maths* in these files can be imported and tested on a
laptop, with no board plugged in. We debugged the steering geometry and the orientation
conversion weeks before the chassis existed.

The sensor files take this one step further, giving each driver its own separate `try`:

```python
try:
    from vl53l0x import VL53L0X
except ImportError:
    VL53L0X = None
```

That is a fix for a real problem. Originally the driver import shared the block above,
so forgetting to copy one driver file to the board set `Pin` to `None` as well and
killed **every** sensor, with no clue why. Now a missing driver costs you that one
sensor and prints its name.

## PICO / servo.py

**Job: turn a steering angle into a signal the servo understands.**

106 lines.

### How a servo actually works

A servo does not listen to voltage or to duty cycle. It listens to **pulse width**.

Every 20 milliseconds it expects one pulse. How long that pulse lasts is the command.
About 1500 microseconds means centre. Longer goes one way, shorter goes the other.

```python
FREQ = 50
PERIOD_US = 1_000_000 // FREQ
```

50 pulses a second is one every 20000 microseconds. `PERIOD_US` is worked out from
`FREQ` rather than typed in, so changing one cannot leave the other wrong.

### The conversion

```python
def steering_to_duty(angle):
    angle = max(-MAX_STEER, min(MAX_STEER, angle))
    pulse_us = CENTRE_US + angle * US_PER_DEGREE * STEER_DIRECTION
    pulse_us = max(MIN_US, min(MAX_US, pulse_us))
    duty = int(pulse_us * 65535 / PERIOD_US)
    return angle, duty
```

Four lines, and each one is defensive.

**Line 1 clamps the angle**, before anything else. `max(-30, min(30, angle))` is the
standard way to force a number into a range in Python. A runaway command of -90 becomes
-30 here.

**Line 2 converts degrees to microseconds.** `US_PER_DEGREE` is 10.5 because an MG90S
sweeps roughly 180 degrees over a pulse range of 500 to 2400 microseconds, and 1900
divided by 180 is about 10.5.

`STEER_DIRECTION` is either 1 or -1. If the servo is mounted mirrored, you set this to
-1 rather than negating the angle wherever it is called. That way the sign convention
stays identical on both boards, which matters because the Pi and the Pico both talk
about steering angles.

**Line 3 clamps the pulse**, a second safety net. Even if `CENTRE_US` were badly
miscalibrated, the servo cannot be driven into the steering linkage. A stalled MG90S
draws over an amp and destroys its own gears in about a minute.

**Line 4 converts to what MicroPython wants.** `duty_u16` takes a number from 0 to 65535
representing the fraction of the frame the pulse is high for. A 1500 microsecond pulse
in a 20000 microsecond frame is 1500/20000 of the time, which is 4915.

**Why the angle is clamped before conversion and not after.** So that the returned angle
and the physical angle always agree. The state machine logs the commanded angle when
something goes wrong, and a log that disagrees with reality is worse than no log.

## PICO / drv8833.py

**Job: turn a speed number into motor PWM.**

101 lines.

### There is no direction pin

This surprises people. A DRV8833 has two inputs per motor, AIN1 and AIN2, and they work
as a pair:

| AIN1 | AIN2 | Motor |
|---|---|---|
| PWM | LOW | forward, speed set by the duty |
| LOW | PWM | reverse |
| LOW | LOW | coasting |
| HIGH | HIGH | braking |

So **which** pin carries the PWM is the direction, and the duty on that pin is the
speed. Same two wires, both jobs.

Since we only drive forward, AIN2 is held low permanently as an ordinary output and AIN1
gets the PWM.

```python
def speed_to_duty(speed):
    speed = max(0, min(MAX_SPEED, speed))
    return speed, int(speed * 65535 / 100)
```

The sign is preserved and the duty comes back positive. On a DRV8833 the direction is
decided by **which** of the two inputs carries the PWM, not by the value on it, so the
caller uses the sign to pick the input and the magnitude to set the speed.

Reverse exists because parallel parking needs it, and so does backing out of a bad park
or recovering when the robot wedges itself.

```python
FREQ = 20_000
```

20 kHz is above human hearing. At 50 Hz or 1 kHz the motor whines audibly, which is
genuinely irritating over a long build session.

## PICO / encoder.py

**Job: count wheel rotations and turn them into distance and speed.**

126 lines.

### What an encoder is

Behind the gearbox, on the motor shaft, is a small magnet disc with two sensors beside
it. As the shaft spins, each sensor produces a square wave. The two sensors are offset,
so their waves are a quarter cycle apart. That offset is what lets you tell direction.

```
forward                    backward
A  __|‾‾|__|‾‾|__          A  __|‾‾|__|‾‾|__
B  ___|‾‾|__|‾‾|_          B  _|‾‾|__|‾‾|___
   B is low when A rises      B is high when A rises
```

### Counting with an interrupt

```python
def on_pulse(pin):
    global pulse_count
    if channel_b.value():
        pulse_count -= DIRECTION_SIGN
    else:
        pulse_count += DIRECTION_SIGN

channel_a.irq(trigger=Pin.IRQ_RISING, handler=on_pulse)
```

An **interrupt** is code that runs the instant a pin changes, pausing whatever else was
happening. `irq` says "when channel A goes from low to high, run this function".

Inside, we look at channel B. If B is already high, we are turning one way; if low, the
other. Four lines, and that is the whole direction problem solved.

**Why an interrupt and not just checking in a loop.** A pulse lasts less than a
millisecond at speed. A loop that pauses to print would miss pulses, and your distance
would quietly read low. Interrupts do not miss.

**Why the handler must stay tiny.** MicroPython forbids allocating memory inside an
interrupt. No printing, no building lists, no floating point that creates new objects.
Integer arithmetic only. That is why all the real maths happens elsewhere.

### From counts to centimetres

```python
PULSES_PER_WHEEL_REV = PULSES_PER_MOTOR_REV * GEAR_RATIO
WHEEL_CIRCUMFERENCE_MM = pi * WHEEL_DIAMETER_MM

rotations = count / PULSES_PER_WHEEL_REV
distance_cm = rotations * WHEEL_CIRCUMFERENCE_MM / 10
```

The encoder counts the *motor* shaft, which spins `GEAR_RATIO` times for each turn of
the wheel. So 7 pulses per motor turn at 1:100 gives 700 pulses per wheel turn.

Then rotations times circumference is distance. Both derived values are calculated from
the two you measure, so changing the wheel diameter automatically updates the
circumference.

`GEAR_RATIO` is the single biggest source of wrong distances, because N20 motors ship
anywhere from 1:30 to 1:298 and you cannot detect it in software.

```python
speed_cm_s = moved_cm * 1000 / delta_ms if delta_ms else 0.0
```

`if delta_ms` guards against dividing by zero if two readings happen in the same
millisecond.

## PICO / motionController.py

**Job: one function for movement.**

127 lines, and most of it is the self test.

```python
def move(speed, steering):
    speed, motor_duty = drv8833.speed_to_duty(speed)
    angle, servo_duty = servo.steering_to_duty(steering)
    motor_pwm.duty_u16(motor_duty)
    steering_pwm.duty_u16(servo_duty)
    return speed, angle
```

All the real work is delegated. The clamping lives in the modules that own each device,
so when you lower `MAX_SPEED` for bench testing, this follows automatically. Two places
that both clamp would eventually disagree.

It returns the **clamped** values, so the caller sees what actually happened rather than
what it asked for.

```python
def stop():
    return move(0, 0)
```

Stop is one line. Zero speed, centred steering, and it inherits all the clamping for
free. Nothing to keep in sync.

Worth knowing: this **coasts** rather than brakes. Both driver inputs end up low, which
is the coast mode from the table earlier, so the robot rolls to a halt.

## PICO / sensors/distance.py

**Job: read four distance sensors that all have the same address.**

203 lines.

### The problem

Every VL53L0X ships with I2C address 0x29, and it cannot be changed by a jumper. Put two
on one bus and both answer at once. The bus garbles and nothing in software can separate
them.

### The multiplexer

A TCA9548A is an eight way electrical switch for the bus. It has one register, a single
byte, where each bit is one channel.

```python
def select_channel(channel):
    i2c.writeto(TCA_ADDRESS, bytes([1 << channel]))
```

`1 << channel` is a **bit shift**. It means "the number 1, moved left by that many
places".

| channel | `1 << channel` | binary |
|---|---|---|
| 0 | 1 | 00000001 |
| 3 | 8 | 00001000 |
| 5 | 32 | 00100000 |

Exactly one bit set means exactly one channel connected, so only one sensor is on the
bus at a time. The others are electrically disconnected and might as well not exist.

The self test checks that only one bit is ever set, and that the bits belonging to the
IMU and the colour sensor are never touched.

### The sensor table

```python
SENSORS = (
    ("front", 0, 0),
    ("left", 3, -45),
    ("right", 4, 45),
    ("rear", 5, 180),
)
```

Name, multiplexer channel, and mounting angle. A **tuple**, not a dictionary, because
MicroPython does not guarantee dictionary order and the printout should always read in
the same order.

This is the only place channel numbers appear. Everything downstream says
`left_distance`, never "channel 3".

### The 45 degree warning

The file's header says it plainly: the left and right sensors are **not** side facing.
They watch the forward diagonals. A reading of D millimetres means roughly 0.71 times D
ahead and the same to the side.

It is written at the top of the file because code that treats a diagonal reading as
lateral clearance will drive the robot into a wall.

### Failing safely

```python
def read_distance(sensor, channel):
    if sensor is None:
        return None
    try:
        select_channel(channel)
        return sensor.read()
    except Exception:
        return None
```

Everything is caught. A sensor that browns out, gets unplugged, or holds the bus must
not take the other three down with it. The caller gets `None`, which every consumer
already handles.

## PICO / sensors/imu.py

**Job: report which way the robot is pointing.**

185 lines.

### What the chip does

The BNO085 contains an accelerometer, a gyroscope and a magnetometer, plus its own
processor running sensor fusion. Unlike a cheaper chip, it hands you finished
orientation rather than raw numbers to filter yourself.

It reports orientation as a **quaternion**, four numbers describing a rotation. You do
not need to understand quaternions to use this code, only to know they avoid a maths
problem that Euler angles have, and that we convert them into ordinary angles.

### The conversion

```python
def quaternion_to_euler(i, j, k, real):
    yaw = atan2(2 * (real * k + i * j), 1 - 2 * (j * j + k * k))
    pitch = asin(max(-1.0, min(1.0, 2 * (real * j - k * i))))
    roll = atan2(2 * (real * i + j * k), 1 - 2 * (i * i + j * j))
    return degrees(yaw) % 360, degrees(pitch), degrees(roll)
```

The three formulas are standard and you can look them up. Two details are ours.

**The clamp inside `asin`.** `asin` only accepts values from -1 to 1. Floating point
rounding can produce 1.0000000001, which raises an error. The clamp prevents a crash
that would only ever happen when the robot is pointing almost straight up. Rare, but a
crash mid run is a crash.

**The `% 360`.** Raw yaw comes out from -180 to +180, so driving north makes it flip
between -179 and +179 frame to frame, which is miserable to write comparisons against.
Wrapping to 0 to 360 gives ordinary compass headings.

### Which report to use

The code enables the rotation vector, which fuses all three sensors including the
magnetometer. **For competition you almost certainly want the game rotation vector
instead**, which ignores the magnetometer.

Your robot has a motor with permanent magnets a few centimetres away, current spikes
through the power traces, and an arena full of metal. The magnetometer will read
nonsense and drag your heading with it. The game rotation vector uses only the
accelerometer and gyroscope, losing absolute north, which you do not need.

### Recovering by itself

```python
if bno is None:
    bno = setup_imu()
    if bno is None:
        print("IMU ERROR")
        sleep_ms(RETRY_MS)
        continue
```

If the sensor disappears, the loop keeps trying to bring it back rather than ending the
run. A knocked cable recovers on its own.

## PICO / sensors/colour.py

**Job: say what colour the mat is underneath the robot.**

208 lines.

### Working in ratios, not raw numbers

The trap is using raw values. They depend on how bright the LED is, how far the sensor
is from the mat, how charged the battery is. Thresholds tuned on the bench at 4 pm fail
in a competition hall at 10 am.

```python
red_ratio = red / clear
```

Dividing by the clear channel gives a value that describes the *surface* rather than the
lighting. A red surface reflects a large share of red no matter how brightly it is lit.

The self test proves it: the same sample at half brightness classifies identically.

### The order of the checks

```python
if clear < BLACK_MAX_CLEAR:
    return "BLACK"
red_ratio = red / clear
```

Black is checked first, on the clear channel alone. Little light coming back means a
dark surface and there is no meaningful colour to extract.

It also means `clear` is guaranteed to be above zero by the time we divide by it. The
order is a safety feature, not just a preference.

### The dominance factor

```python
DOMINANCE = 1.4
if (red_ratio >= RED_MIN_RATIO
        and red_ratio > green_ratio * DOMINANCE
        and red_ratio > blue_ratio * DOMINANCE):
    return "RED"
```

Red must not merely be the largest, it must beat the others by 40 percent. Without this,
a slightly warm shade of grey reports as RED. A surface has to genuinely be red.

If nothing wins, the answer is `"UNKNOWN"`, which is a real answer and better than
guessing.

## PICO / sensors/sensorManager.py

**Job: gather every reading into one dictionary.**

217 lines.

```python
return {
    "front_distance": ranges["front"],
    ...
    "heading": heading,
    "distance_travelled": distance_travelled,
    "floor_colour": floor_colour,
}
```

Anything that wants to know about the robot asks one function and gets one dictionary.
Nothing above this layer knows that a multiplexer exists, or that the IMU speaks
quaternions.

Any sensor that cannot be read contributes `None` rather than raising, so a dead sensor
costs you one value instead of the whole sweep.

### One bus, one object

```python
if distance.i2c is not None:
    imu.i2c = colour.i2c = distance.i2c
```

Each sensor module creates its own I2C object when imported, which is correct when you
run that file alone on the bench. Here all three share the same physical wires, so they
are handed a single bus to talk over.

## PICO / main.py

**Job: receive commands, drive the robot, report back.**

190 lines. This runs automatically when the Pico powers up, because MicroPython always
runs a file called `main.py` at boot.

### Two speeds in one loop

```python
LOOP_MS = 20
SENSOR_INTERVAL_MS = 150
```

Commands are read and applied **every** pass, 50 times a second, because a command that
waits is a robot that has already driven somewhere else.

A full sweep of four distance sensors costs about 130 milliseconds, mostly because each
VL53L0X takes 33 milliseconds to measure. So sensors run on their own slower clock and
never delay the motor. A single speed loop would have added 130 milliseconds of lag to
every steering command.

### Newest command wins

```python
while b"\n" in buffer:
    line, buffer = buffer.split(b"\n", 1)
    parsed = parse_command(line)
    if parsed:
        command = parsed
```

Every complete line is read but only the last valid one is kept. A queued command is
already stale.

Note that this reads into a buffer rather than using `readline`. A serial read can hand
you half a line, or three lines at once. Accumulating and splitting on the newline means
a message split across two reads still arrives intact.

### The watchdog

```python
elif not stopped and ticks_diff(now, last_command_ms) > COMMAND_TIMEOUT_MS:
    motionController.stop()
    stopped = True
```

If no command arrives for half a second, the robot stops.

**This is the single most important reason the control system is split across two
boards.** If the Pi crashes, the Pico notices and stops. If the Pi drove the motor
directly, a crash would leave the robot running at whatever speed it was last told,
until it hit something.

`stopped` is a latch, so once we have stopped we do not repeatedly send stop commands or
re-apply the old one.

### The message format

```python
def state_line(state):
    line = "%s,%d,%d,%d,%d" % (STATE_PREFIX, mm(front), mm(left), mm(right), mm(rear))
    if state["heading"] is not None:
        line += ",%.1f" % state["heading"]
    return line + "\n"
```

Distances in millimetres, with -1 meaning a sensor had no reading.

Heading is **left out entirely** when unknown rather than sent as a fake value, because
the Pi's parser already treats a missing field as unknown. There is no bogus number that
could be mistaken for a real heading.

The self test for this file loads the **Pi's actual parser** and checks a line
round trips through it. Not a copy of the parser, the real one. That is how we know both
boards agree on the format.

## PICO / uart_echo.py

**Job: prove the serial link works, and nothing else.**

46 lines. A bench tool. It prints whatever arrives and ignores anything malformed. Use it
before connecting motors, so that when something does not work you know whether the
problem is the link or the robot.

## PICO / deploy.sh

**Job: copy the code to the board.**

A shell script, not Python.

```sh
$MPREMOTE cp main.py motionController.py servo.py drv8833.py encoder.py :
$MPREMOTE cp sensors/sensorManager.py sensors/distance.py ... :
```

The trailing `:` means "the board". Notice everything lands **flat** at the root, even
though this repository keeps sensors in a subfolder. MicroPython's `import distance`
does not know about folders, so the repository layout and the board layout are
deliberately different. This script is what makes that safe to forget.

