"""Competition State Machine: what the robot should be doing right now.

    machine.update(vision_data, robot_state, navigation_output, now)
        -> {"state": "FOLLOW_COURSE", "reason": "...", "events": [...]}

This decides WHAT. The Navigation Engine decides HOW. Nothing here calculates
steering, drives a motor, or looks at a camera image.

The design is event driven, in two separable halves:

    detect_events(context)  turns raw sensor and vision readings into named
                            events - CORNER_DETECTED, RED_PILLAR, and so on.
                            This is the half that knows about millimetres.

    TRANSITIONS             a table of "in this state, this event means go to
                            that state". This is the half that knows about
                            behaviour, and it contains no numbers at all.

Change the field layout or improve the navigation and you edit detect_events or
a constant. The transition table only changes when the BEHAVIOUR changes.

Run:  python state_machine.py             simulate a full run
      python state_machine.py --selftest
"""

import sys
import time
from enum import Enum

import mission_manager
import navigation_engine

# ============================================================================
# CONFIGURATION - every tunable number in the file lives here
# ============================================================================

# --- Pillars ----------------------------------------------------------------
# One definition, owned by the navigation engine, so the two cannot disagree
# about what counts as a real pillar.
MIN_PILLAR_AREA = navigation_engine.MIN_PILLAR_AREA
PILLAR_CLEAR_TIME_S = 0.5       # pillar must be gone this long before resuming
PASS_DISTANCE_MM = 350          # closer than this and we are committed to the pass

# --- Corners ----------------------------------------------------------------
CORNER_TRIGGER_MM = 600         # wall this close ahead means a corner is due
CAMERA_CORNER_MM = 700          # the camera sees the wall before the ToF confirms it
# A corner announces itself by one side wall running out. This fires while the
# robot is still approaching, where a front distance reading only fires once it
# is nearly there.
WALL_COLLAPSE_BALANCE = 0.75    # this one sided means one wall has ended
WALL_COLLAPSE_MIN_PIXELS = 200  # ...and enough wall remains on the other side
CORNER_ANGLE_DEG = 80           # heading change that counts as the turn done
CORNERS_PER_LAP = 4
LAPS_PER_RUN = 3
CORNERS_PER_RUN = CORNERS_PER_LAP * LAPS_PER_RUN

LANE_CENTRED_BAND = 0.10        # lane offset this small counts as back on line

# --- Parking ----------------------------------------------------------------
# Three laps done, back where we started: park between the two magenta markers.
PARKING_ALIGNED_BAND = 0.12     # slot this close to centre means lined up
# The camera faces forward, so the slot is only visible while driving at it -
# which means we enter nose first, and it is the FRONT sensor that closes as we
# go in. The rear sensor is pointed back at the open mat we came from and would
# never trigger. Lower than navigation's emergency stop on purpose; the speed
# floor below is what lets us creep past that to reach it.
PARKING_STOP_MM = 100
PARKING_STATES = None           # filled in below, once State exists

# --- Recovery ---------------------------------------------------------------
RECOVERY_CLEAR_MM = 400         # this much room ahead means we are out of trouble
MAX_RECOVERY_ATTEMPTS = 3       # after this many, stop rather than thrash

# --- Stuck detection --------------------------------------------------------
STUCK_SPEED_CM_S = 2.0          # measured speed below this counts as not moving
STUCK_TIME_S = 2.0              # ...for this long, while being told to drive

DEBUG = True                    # print every state change


class State(Enum):
    """Add a state by adding a member, a timeout if it needs one, and a row in
    TRANSITIONS. Nothing else in the file changes."""
    WAIT_FOR_START = "WAIT_FOR_START"
    INITIALISE = "INITIALISE"
    FOLLOW_COURSE = "FOLLOW_COURSE"
    APPROACH_PILLAR = "APPROACH_PILLAR"
    PASS_PILLAR = "PASS_PILLAR"
    RECENTER = "RECENTER"
    TURN_CORNER = "TURN_CORNER"
    SEARCH_PARKING = "SEARCH_PARKING"
    ALIGN_PARKING = "ALIGN_PARKING"
    ENTER_PARKING = "ENTER_PARKING"
    RECOVERY = "RECOVERY"
    FINISHED = "FINISHED"


class Event(Enum):
    """Something the robot noticed. Events are reported whenever they are true,
    whatever state we are in - the table decides which ones matter."""
    START_SIGNAL = "START_SIGNAL"
    STATE_TIMEOUT = "STATE_TIMEOUT"
    RED_PILLAR = "RED_PILLAR"
    GREEN_PILLAR = "GREEN_PILLAR"
    PILLAR_CLOSE = "PILLAR_CLOSE"
    PILLAR_PASSED = "PILLAR_PASSED"
    LANE_CENTRED = "LANE_CENTRED"
    CORNER_AHEAD = "CORNER_AHEAD"
    CORNER_DETECTED = "CORNER_DETECTED"
    CORNER_COMPLETE = "CORNER_COMPLETE"
    PATH_CLEAR = "PATH_CLEAR"
    RECOVERY_REQUIRED = "RECOVERY_REQUIRED"
    RECOVERY_EXHAUSTED = "RECOVERY_EXHAUSTED"
    RUN_COMPLETE = "RUN_COMPLETE"
    PARKING_VISIBLE = "PARKING_VISIBLE"
    PARKING_ALIGNED = "PARKING_ALIGNED"
    PARKED = "PARKED"


# Only these states may decide we have arrived. Elsewhere a close wall is a
# corner or an obstacle, not a parking space.
PARKING_STATES = (State.ALIGN_PARKING, State.ENTER_PARKING)

# How long each state may last before STATE_TIMEOUT is reported. A state absent
# from this table never times out.
STATE_TIMEOUT_S = {
    State.INITIALISE: 1.0,      # settle sensors and zero the counters
    State.APPROACH_PILLAR: 6.0,  # still approaching after this long? something is wrong
    State.PASS_PILLAR: 4.0,      # a pass takes a second or two, not four
    State.RECENTER: 3.0,         # give up realigning and just drive
    State.SEARCH_PARKING: 20.0,  # a lap of hunting is plenty
    State.ALIGN_PARKING: 8.0,
    State.ENTER_PARKING: 6.0,
    State.TURN_CORNER: 4.0,     # a turn taking longer than this has failed
    State.RECOVERY: 3.0,
}

# The whole behaviour of the robot, in one table. Rows are checked in order, so
# the first matching event wins - that ordering is the priority of the state.
TRANSITIONS = {
    State.WAIT_FOR_START: (
        (Event.START_SIGNAL, State.INITIALISE),
    ),
    State.INITIALISE: (
        (Event.STATE_TIMEOUT, State.FOLLOW_COURSE),
    ),
    State.FOLLOW_COURSE: (
        (Event.RUN_COMPLETE, State.SEARCH_PARKING),
        (Event.RECOVERY_REQUIRED, State.RECOVERY),
        (Event.CORNER_DETECTED, State.TURN_CORNER),     # a wall beats a pillar
        (Event.RED_PILLAR, State.APPROACH_PILLAR),
        (Event.GREEN_PILLAR, State.APPROACH_PILLAR),
    ),
    State.APPROACH_PILLAR: (
        (Event.STATE_TIMEOUT, State.RECOVERY),
        (Event.CORNER_DETECTED, State.TURN_CORNER),     # a corner still wins
        (Event.PILLAR_CLOSE, State.PASS_PILLAR),
        (Event.PILLAR_PASSED, State.RECENTER),          # lost sight of it early
    ),
    State.PASS_PILLAR: (
        # No recovery here: losing sight of a pillar you are squeezing past is
        # exactly what success looks like, so both exits lead to recentring.
        (Event.PILLAR_PASSED, State.RECENTER),
        (Event.STATE_TIMEOUT, State.RECENTER),
    ),
    State.RECENTER: (
        (Event.RUN_COMPLETE, State.SEARCH_PARKING),
        (Event.CORNER_DETECTED, State.TURN_CORNER),
        (Event.LANE_CENTRED, State.FOLLOW_COURSE),
        (Event.STATE_TIMEOUT, State.FOLLOW_COURSE),     # good enough, carry on
    ),
    State.TURN_CORNER: (
        # Align to the new straight before looking for pillars again.
        (Event.CORNER_COMPLETE, State.RECENTER),
        (Event.STATE_TIMEOUT, State.RECOVERY),
    ),
    State.SEARCH_PARKING: (
        (Event.PARKING_VISIBLE, State.ALIGN_PARKING),
        (Event.CORNER_DETECTED, State.TURN_CORNER),     # keep driving the lap
        (Event.STATE_TIMEOUT, State.FINISHED),          # never found it, stop safely
    ),
    State.ALIGN_PARKING: (
        (Event.PARKED, State.FINISHED),
        (Event.PARKING_ALIGNED, State.ENTER_PARKING),
        (Event.STATE_TIMEOUT, State.SEARCH_PARKING),    # lost it, go round again
    ),
    State.ENTER_PARKING: (
        (Event.PARKED, State.FINISHED),
        (Event.STATE_TIMEOUT, State.FINISHED),          # close enough, stop anyway
    ),
    State.RECOVERY: (
        (Event.PATH_CLEAR, State.FOLLOW_COURSE),
        (Event.RECOVERY_EXHAUSTED, State.FINISHED),
        (Event.STATE_TIMEOUT, State.FOLLOW_COURSE),     # out of time, try again
    ),
    State.FINISHED: (),                                 # terminal
}

# ============================================================================
# MISSIONS - which behaviours belong to which round
# ============================================================================
# Every mission needs these four: you always wait, always initialise, always
# want a way out of trouble, and always end. They are not optional.
CORE_STATES = frozenset((State.WAIT_FOR_START, State.INITIALISE,
                         State.RECOVERY, State.FINISHED))

MISSION_STATES = {
    mission_manager.Mission.OPEN_CHALLENGE.value: CORE_STATES | frozenset((
        State.FOLLOW_COURSE, State.TURN_CORNER)),

    mission_manager.Mission.OBSTACLE_CHALLENGE.value: CORE_STATES | frozenset((
        State.FOLLOW_COURSE, State.APPROACH_PILLAR, State.PASS_PILLAR,
        State.RECENTER, State.TURN_CORNER,
        # The obstacle round ends by parking between the magenta markers.
        State.SEARCH_PARKING, State.ALIGN_PARKING, State.ENTER_PARKING)),

    mission_manager.Mission.PARKING.value: CORE_STATES | frozenset((
        State.SEARCH_PARKING, State.ALIGN_PARKING, State.ENTER_PARKING)),
}

# What a mission does instead, when the table points at a state it does not
# have. None means the transition simply does not apply and the next row is
# tried. Without this an open-challenge robot would finish its laps, be told to
# go and park, find no parking state, and drive on forever.
MISSION_SUBSTITUTE = {
    mission_manager.Mission.OPEN_CHALLENGE.value: {
        State.APPROACH_PILLAR: None,                # no pillars in this round
        State.PASS_PILLAR: None,
        State.RECENTER: State.FOLLOW_COURSE,
        State.SEARCH_PARKING: State.FINISHED,       # laps done, and that is the run
    },
    mission_manager.Mission.OBSTACLE_CHALLENGE.value: {},   # uses every state

    mission_manager.Mission.PARKING.value: {
        State.FOLLOW_COURSE: State.SEARCH_PARKING,  # parking starts by looking
        State.RECENTER: State.SEARCH_PARKING,
        State.APPROACH_PILLAR: None,
        State.PASS_PILLAR: None,
        State.TURN_CORNER: None,
    },
}


def allowed_state(state, mission):
    """Where this transition actually leads in this mission.

    The state itself when the mission has it, a substitute when the mission has
    something else in mind, or None when the transition does not apply at all.
    """
    if state in MISSION_STATES[mission]:
        return state
    return MISSION_SUBSTITUTE[mission].get(state)


EVENT_REASON = {
    Event.START_SIGNAL: "Start signal received",
    Event.STATE_TIMEOUT: "Timed out",
    Event.RED_PILLAR: "Red pillar detected",
    Event.GREEN_PILLAR: "Green pillar detected",
    Event.PILLAR_CLOSE: "Pillar close, committing to the pass",
    Event.PILLAR_PASSED: "Pillar passed",
    Event.LANE_CENTRED: "Back on the centre line",
    Event.CORNER_AHEAD: "Wall visible ahead",
    Event.CORNER_DETECTED: "Corner detected",
    Event.CORNER_COMPLETE: "Corner complete",
    Event.PATH_CLEAR: "Path clear again",
    Event.RECOVERY_REQUIRED: "Commanded to drive but not moving",
    Event.RECOVERY_EXHAUSTED: "Recovery failed too many times, stopping",
    Event.RUN_COMPLETE: "%d laps complete, looking for the parking slot" % LAPS_PER_RUN,
    Event.PARKING_VISIBLE: "Both parking markers in view",
    Event.PARKING_ALIGNED: "Lined up with the slot",
    Event.PARKED: "Parked",
}


# --- What each state does with the navigation command -----------------------
# Navigation always decides HOW to drive. The state machine may only restrain
# it, never invent a command of its own. Speed cap per state; None means "take
# whatever navigation asked for". A table, so a new state cannot quietly forget
# to declare its behaviour - the selftest checks every state appears here.
STATE_SPEED_CAP = {
    State.WAIT_FOR_START: 0,
    State.INITIALISE: 0,
    State.FOLLOW_COURSE: None,
    State.APPROACH_PILLAR: None,    # avoiding IS navigation's job, do not clip it
    State.PASS_PILLAR: None,
    State.RECENTER: None,
    State.SEARCH_PARKING: None,     # navigation crawls for all three of these
    State.ALIGN_PARKING: None,
    State.ENTER_PARKING: None,
    State.TURN_CORNER: 35,          # corners are tighter than the straights
    State.RECOVERY: 30,             # crawl while working out where we are
    State.FINISHED: 0,
}

# Minimum speed per state; None means "navigation may stop us".
#
# TURN_CORNER needs one. Turning towards a wall, the front sensor keeps closing,
# so navigation's emergency stop fires mid-turn - and a stopped robot stops
# turning, so the corner never completes, the state times out into RECOVERY,
# and RECOVERY cannot clear either because the wall is still there. The robot
# gives up on a corner it was in the middle of taking correctly.
#
# The trade: while cornering we will creep forward even with something close
# ahead. That something is the wall we are turning away from, which is the whole
# point of the manoeuvre. Keep the floor low.
STATE_SPEED_FLOOR = {
    State.WAIT_FOR_START: None,
    State.INITIALISE: None,
    State.FOLLOW_COURSE: None,
    State.APPROACH_PILLAR: None,
    State.PASS_PILLAR: None,
    State.RECENTER: None,
    State.SEARCH_PARKING: None,
    State.ALIGN_PARKING: None,
    # Same reasoning as the corner: navigation stops for the wall ahead, but
    # that wall is the back of the parking space. Creep in and let PARKED stop us.
    State.ENTER_PARKING: 15,
    State.TURN_CORNER: 20,
    State.RECOVERY: None,
    State.FINISHED: None,
}

# States that hold the wheels straight whatever navigation wants.
STATE_HOLDS_STRAIGHT = (State.WAIT_FOR_START, State.INITIALISE, State.FINISHED)


def apply_state(state, navigation_output):
    """The final command: navigation's request, restrained by the current state.

    Speed is signed, so the limits are applied to how fast we are going rather
    than to the number itself. Taking `max(speed, floor)` on a reversing robot
    would turn a gentle reverse into a forward lurch, which is exactly the kind
    of mistake that only shows up once something asks to go backwards.
    """
    navigation_output = navigation_output or {}
    speed = navigation_output.get("speed", 0)
    direction = -1 if speed < 0 else 1
    magnitude = abs(speed)

    # Floor first, then cap - so a cap of 0 always wins and FINISHED really stops.
    floor = STATE_SPEED_FLOOR[state]
    if floor is not None:
        magnitude = max(magnitude, floor)
    cap = STATE_SPEED_CAP[state]
    if cap is not None:
        magnitude = min(magnitude, cap)

    steering = (0 if state in STATE_HOLDS_STRAIGHT
                else navigation_output.get("steering", 0))
    return int(direction * magnitude), int(steering)


def heading_change(start, current):
    """Signed degrees turned, -180 to +180, wrapping cleanly through zero.

    Without the wrap, a robot turning from 350 to 10 degrees looks like it spun
    340 degrees backwards instead of 20 degrees forwards.
    """
    if start is None or current is None:
        return 0.0
    return (current - start + 180) % 360 - 180


def detect_events(context):
    """Everything that is true right now, as a list of events.

    Pure: it reads the context and nothing else. Events are reported whether or
    not the current state cares, which is what keeps this function independent
    of the transition table - and means a new state can consume an event that
    already exists without touching this code.
    """
    events = []

    if context["started"]:
        events.append(Event.START_SIGNAL)

    timeout = STATE_TIMEOUT_S.get(context["state"])
    if timeout is not None and context["time_in_state"] >= timeout:
        events.append(Event.STATE_TIMEOUT)

    if context["corners_turned"] >= CORNERS_PER_RUN:
        events.append(Event.RUN_COMPLETE)

    if context["stuck"]:
        events.append(Event.RECOVERY_REQUIRED)
    if context["recovery_attempts"] > MAX_RECOVERY_ATTEMPTS:
        events.append(Event.RECOVERY_EXHAUSTED)

    # The camera sees the wall first; the front ToF confirms it. Either alone is
    # enough, so a dead front sensor still corners and a blinded camera still
    # corners - but when both work, the ToF is what commits us.
    wall = context["wall_distance"]
    if wall is not None and wall < CAMERA_CORNER_MM:
        events.append(Event.CORNER_AHEAD)

    front = context["front_distance"]
    if front is not None:
        if front < CORNER_TRIGGER_MM:
            events.append(Event.CORNER_DETECTED)
        if front >= RECOVERY_CLEAR_MM:
            events.append(Event.PATH_CLEAR)
    elif Event.CORNER_AHEAD in events:
        events.append(Event.CORNER_DETECTED)        # no ToF, trust the camera

    walls = context["walls"]
    if walls and walls["left"] + walls["right"] >= WALL_COLLAPSE_MIN_PIXELS:
        if abs(walls["balance"]) >= WALL_COLLAPSE_BALANCE:
            events.append(Event.CORNER_AHEAD)

    lane = context["lane_offset"]
    if lane is not None and abs(lane) < LANE_CENTRED_BAND:
        events.append(Event.LANE_CENTRED)

    # Both markers must be in view before there is a slot to aim at - one
    # marker tells you nothing about where the gap is.
    parking = context["parking"]
    if parking and parking.get("markers") == 2:
        events.append(Event.PARKING_VISIBLE)
        if abs(parking["offset"]) < PARKING_ALIGNED_BAND:
            events.append(Event.PARKING_ALIGNED)

    if (context["state"] in PARKING_STATES and front is not None
            and front < PARKING_STOP_MM):
        events.append(Event.PARKED)

    pillar = context["pillar"]
    if pillar:
        events.append(Event.RED_PILLAR if pillar["colour"] == navigation_engine.COLOUR_RED
                      else Event.GREEN_PILLAR)
        distance = pillar.get("distance")
        if distance is not None and distance < PASS_DISTANCE_MM:
            events.append(Event.PILLAR_CLOSE)
    elif context["pillar_gone_for"] >= PILLAR_CLEAR_TIME_S:
        # One dropped frame while squeezing past a pillar must not end the
        # manoeuvre, so the pillar has to stay gone before this is reported.
        events.append(Event.PILLAR_PASSED)

    if abs(heading_change(context["corner_start_heading"],
                          context["heading"])) >= CORNER_ANGLE_DEG:
        events.append(Event.CORNER_COMPLETE)

    return events


class StateMachine:
    def __init__(self, mission=None):
        # The Mission Manager says which round we are running, and that decides
        # which behaviours exist. It never says how to drive.
        self.mission = mission or mission_manager.current_mission()["mission"]
        self.state = State.WAIT_FOR_START
        self.reason = "Waiting for the start signal"
        self.events = []
        self.corners_turned = 0
        self.recovery_attempts = 0
        self._started = False
        self._state_since = 0.0
        self._heading = None
        self._corner_start_heading = None
        self._pillar_lost_since = None
        self._stuck_since = None

    def start(self):
        """The start signal. Nothing moves until this is called."""
        self._started = True

    def update(self, vision_data=None, robot_state=None,
               navigation_output=None, now=None):
        """Advance one step and report the current behaviour."""
        now = time.monotonic() if now is None else now

        context = self._read_inputs(vision_data, robot_state, navigation_output, now)
        self.events = detect_events(context)

        # First event in this state's row that happened AND leads somewhere this
        # mission has. A row pointing at a state the mission lacks is skipped.
        for event, next_state in TRANSITIONS[self.state]:
            if event not in self.events:
                continue
            target = allowed_state(next_state, self.mission)
            if target is None:
                continue
            self._change_to(target, event, context)
            break

        speed, steering = apply_state(self.state, navigation_output)
        return {"state": self.state.value, "reason": self.reason,
                "mission": self.mission,
                "events": [event.value for event in self.events],
                "speed": speed, "steering": steering}

    def _read_inputs(self, vision_data, robot_state, navigation_output, now):
        """Turn the raw module outputs into the facts detect_events works from."""
        robot_state = robot_state or {}
        navigation_output = navigation_output or {}

        # Reuse the navigation engine's own idea of which pillar matters, so
        # the state machine and the steering never disagree about what is ahead.
        pillar = navigation_engine.select_pillar(
            navigation_engine.normalise_pillars(vision_data))

        self._heading = robot_state.get("heading")

        # Reuse navigation's own lane maths rather than repeating it here, so
        # the two modules can never disagree about where the centre line is.
        lane = navigation_engine.lane_offset(robot_state.get("left_distance"),
                                             robot_state.get("right_distance"))

        # Being told to drive while not actually moving means we are wedged.
        measured_speed = robot_state.get("speed")
        stalled = (navigation_output.get("speed", 0) > 0
                   and measured_speed is not None
                   and measured_speed < STUCK_SPEED_CM_S)
        if not stalled:
            self._stuck_since = None
        elif self._stuck_since is None:
            # `or` would be wrong here: a timestamp of 0.0 is falsy but set.
            self._stuck_since = now

        if pillar:
            self._pillar_lost_since = None
        elif self._pillar_lost_since is None:
            self._pillar_lost_since = now

        return {
            "now": now,
            "state": self.state,
            "time_in_state": now - self._state_since,
            "started": self._started,
            "pillar": pillar,
            "pillar_gone_for": (0.0 if pillar
                                else now - self._pillar_lost_since),
            "front_distance": robot_state.get("front_distance"),
            "wall_distance": (vision_data.get("wall_distance")
                              if isinstance(vision_data, dict) else None),
            "parking": (vision_data.get("parking")
                        if isinstance(vision_data, dict) else None),
            "walls": (vision_data.get("walls")
                      if isinstance(vision_data, dict) else None),
            "rear_distance": robot_state.get("rear_distance"),
            "lane_offset": lane,
            "heading": self._heading,
            "corner_start_heading": self._corner_start_heading,
            "corners_turned": self.corners_turned,
            "recovery_attempts": self.recovery_attempts,
            "stuck": (self._stuck_since is not None
                      and now - self._stuck_since >= STUCK_TIME_S),
        }

    def _change_to(self, new_state, event, context):
        """The only place the state is allowed to change."""
        if DEBUG:
            print("STATE CHANGE")
            print("  Old State: %s" % self.state.value)
            print("  New State: %s" % new_state.value)
            print("  Event:     %s" % event.value)
            print("  Reason:    %s" % EVENT_REASON[event])

        # What the event itself means, regardless of where we end up.
        if event is Event.CORNER_COMPLETE:
            self.corners_turned += 1
            self._corner_start_heading = None

        # What the state we are entering needs set up for it.
        if new_state is State.INITIALISE:
            self.corners_turned = 0
            self.recovery_attempts = 0
        elif new_state is State.TURN_CORNER:
            self._corner_start_heading = context["heading"]
        elif new_state is State.APPROACH_PILLAR:
            self._pillar_lost_since = None
        elif new_state is State.RECOVERY:
            self.recovery_attempts += 1

        self.state = new_state
        self.reason = EVENT_REASON[event]
        self._state_since = context["now"]
        self._stuck_since = None


# ============================================================================
def simulate():
    """A scripted run, so the transitions can be checked without a robot."""
    machine = StateMachine()
    clock = [0.0]

    def step(seconds, vision=None, front=900, heading=0.0,
             commanded_speed=45, measured_speed=40.0):
        clock[0] += seconds
        return machine.update(
            vision, {"front_distance": front, "heading": heading,
                     "speed": measured_speed},
            {"speed": commanded_speed}, now=clock[0])

    red = {"pillars": [{"colour": "RED", "x": 415, "area": 8200, "distance": 600}]}

    print("=== waiting ===")
    step(0.1)
    machine.start()
    step(0.1)
    step(STATE_TIMEOUT_S[State.INITIALISE])

    print("\n=== pillar ===")
    step(0.2, red)
    step(0.2, red)
    step(0.2, {"pillars": [{"colour": "RED", "x": 415, "area": 8200,
                            "distance": PASS_DISTANCE_MM - 50}]})   # committed
    step(0.2)                                   # pillar gone
    step(PILLAR_CLEAR_TIME_S)                   # confirmed passed
    step(0.2, front=900, heading=0.0)           # recentring

    print("\n=== twelve corners ===")
    heading = 0.0
    for _ in range(CORNERS_PER_RUN):
        step(0.2, front=CORNER_TRIGGER_MM - 100, heading=heading)   # wall ahead
        heading = (heading + CORNER_ANGLE_DEG + 5) % 360            # turn made
        step(0.5, heading=heading)
        step(0.2, heading=heading)

    print("\n=== final state: %s ===" % machine.update(now=clock[0])["state"])


def selftest():
    # --- heading wrap, the part most likely to be got wrong ---
    assert heading_change(0, 90) == 90
    assert heading_change(350, 10) == 20            # forwards through zero
    assert heading_change(10, 350) == -20           # and backwards
    assert abs(heading_change(0, 180)) == 180       # half a turn, either sign
    assert heading_change(None, 90) == 0 and heading_change(0, None) == 0

    # --- the table itself must be well formed ---
    assert set(TRANSITIONS) == set(State), "a state has no transition row"
    for state, rows in TRANSITIONS.items():
        for event, target in rows:
            assert isinstance(event, Event) and isinstance(target, State), (state, event)
            assert event in EVENT_REASON, event
        events = [event for event, _ in rows]
        assert len(events) == len(set(events)), "duplicate event in %s" % state

    # --- events are detected independently of any state ---
    base = {"now": 0.0, "state": State.FOLLOW_COURSE, "time_in_state": 0.0,
            "started": False, "pillar": None, "pillar_gone_for": 0.0,
            "front_distance": 900, "wall_distance": None, "lane_offset": None,
            "parking": None, "walls": None, "rear_distance": None,
            "heading": 0.0, "corner_start_heading": None,
            "corners_turned": 0, "recovery_attempts": 0, "stuck": False}

    def events_for(**changes):
        context = dict(base)
        context.update(changes)
        return detect_events(context)

    assert Event.PATH_CLEAR in events_for()
    assert Event.CORNER_DETECTED in events_for(front_distance=CORNER_TRIGGER_MM - 1)
    assert Event.CORNER_DETECTED not in events_for(front_distance=CORNER_TRIGGER_MM)
    assert Event.RED_PILLAR in events_for(pillar={"colour": "RED"})
    assert Event.GREEN_PILLAR in events_for(pillar={"colour": "GREEN"})
    assert Event.PILLAR_PASSED in events_for(pillar_gone_for=PILLAR_CLEAR_TIME_S)
    assert Event.PILLAR_PASSED not in events_for(pillar_gone_for=0.1)
    assert Event.RECOVERY_REQUIRED in events_for(stuck=True)
    assert Event.PILLAR_CLOSE in events_for(
        pillar={"colour": "RED", "distance": PASS_DISTANCE_MM - 1})
    assert Event.PILLAR_CLOSE not in events_for(
        pillar={"colour": "RED", "distance": PASS_DISTANCE_MM + 1})
    assert Event.PILLAR_CLOSE not in events_for(pillar={"colour": "RED"})   # no range
    assert Event.LANE_CENTRED in events_for(lane_offset=0.0)
    assert Event.LANE_CENTRED not in events_for(lane_offset=0.5)
    assert Event.LANE_CENTRED not in events_for(lane_offset=None)   # cannot tell
    assert Event.CORNER_AHEAD in events_for(wall_distance=CAMERA_CORNER_MM - 1)
    # a corner also announces itself when one side wall runs out
    assert Event.CORNER_AHEAD in events_for(
        walls={"left": 900, "right": 20, "balance": 0.95})
    # ...but a wall on both sides is a straight, not a corner
    assert Event.CORNER_AHEAD not in events_for(
        walls={"left": 500, "right": 500, "balance": 0.0})
    # ...and neither is an empty frame, however one sided the balance looks
    assert Event.CORNER_AHEAD not in events_for(
        walls={"left": 30, "right": 0, "balance": 1.0})
    # the camera alone corners when the front ToF is dead...
    blind = events_for(front_distance=None, wall_distance=CAMERA_CORNER_MM - 1)
    assert Event.CORNER_DETECTED in blind
    # ...and the ToF alone corners when the camera sees no wall
    assert Event.CORNER_DETECTED in events_for(front_distance=CORNER_TRIGGER_MM - 1)
    # but a distant wall on camera with clear space ahead is not a corner
    assert Event.CORNER_DETECTED not in events_for(wall_distance=CAMERA_CORNER_MM + 100)

    # one marker is not a slot - you cannot aim at a gap you can only half see
    assert Event.PARKING_VISIBLE not in events_for(
        parking={"markers": 1, "offset": 0.0})
    assert Event.PARKING_VISIBLE in events_for(parking={"markers": 2, "offset": 0.5})
    assert Event.PARKING_ALIGNED not in events_for(
        parking={"markers": 2, "offset": 0.5})
    assert Event.PARKING_ALIGNED in events_for(parking={"markers": 2, "offset": 0.0})
    # arriving is the FRONT sensor closing, and only counts while parking -
    # everywhere else a close wall is a corner or an obstacle
    assert Event.PARKED in events_for(state=State.ENTER_PARKING,
                                      front_distance=PARKING_STOP_MM - 1)
    assert Event.PARKED in events_for(state=State.ALIGN_PARKING,
                                      front_distance=PARKING_STOP_MM - 1)
    assert Event.PARKED not in events_for(state=State.ENTER_PARKING,
                                          front_distance=PARKING_STOP_MM + 1)
    assert Event.PARKED not in events_for(state=State.FOLLOW_COURSE,
                                          front_distance=PARKING_STOP_MM - 1)
    assert Event.PARKED not in events_for(state=State.ENTER_PARKING,
                                          front_distance=None)
    # and the floor must let us creep past navigation's emergency stop to get there
    assert STATE_SPEED_FLOOR[State.ENTER_PARKING] > 0
    assert PARKING_STOP_MM < navigation_engine.STOP_ENTER_MM
    assert Event.RUN_COMPLETE in events_for(corners_turned=CORNERS_PER_RUN)
    assert Event.CORNER_COMPLETE in events_for(corner_start_heading=0.0,
                                               heading=float(CORNER_ANGLE_DEG))
    assert Event.CORNER_COMPLETE in events_for(corner_start_heading=350.0, heading=75.0)
    # a state with no timeout never reports one, however long it sits there
    assert Event.STATE_TIMEOUT not in events_for(state=State.FOLLOW_COURSE,
                                                 time_in_state=9999)
    assert Event.STATE_TIMEOUT in events_for(state=State.TURN_CORNER,
                                             time_in_state=9999)

    # --- and unhandled events are simply ignored by the current state ---
    following = StateMachine()
    following.state = State.FOLLOW_COURSE
    result = following.update(None, {"front_distance": 900, "heading": 90.0,
                                     "speed": 40.0}, {"speed": 45}, now=0)
    assert "PATH_CLEAR" in result["events"]          # reported...
    assert result["state"] == "FOLLOW_COURSE"        # ...and correctly ignored

    def machine_at(state, **attributes):
        machine = StateMachine()
        machine.start()
        machine.state = state
        machine.__dict__.update(attributes)
        return machine

    clear = {"front_distance": 900, "heading": 0.0, "speed": 40.0}
    close = {"front_distance": CORNER_TRIGGER_MM - 100, "heading": 0.0, "speed": 40.0}
    blocked = {"front_distance": 100, "heading": 0.0, "speed": 0.0}
    driving = {"speed": 45}
    red = {"pillars": [{"colour": "RED", "x": 415, "area": 8200, "distance": 600}]}

    # --- nothing happens before the start signal ---
    idle = StateMachine()
    for _ in range(5):
        assert idle.update(None, clear, driving, now=0)["state"] == "WAIT_FOR_START"
    idle.start()
    assert idle.update(None, clear, driving, now=0)["state"] == "INITIALISE"
    assert idle.update(None, clear, driving,
                       now=STATE_TIMEOUT_S[State.INITIALISE])["state"] == "FOLLOW_COURSE"

    # --- the main loop of the run ---
    following = machine_at(State.FOLLOW_COURSE)
    assert following.update(None, clear, driving, now=0)["state"] == "FOLLOW_COURSE"
    assert following.update(red, clear, driving, now=0)["state"] == "APPROACH_PILLAR"

    # approach -> commit once it is close
    near_red = {"pillars": [{"colour": "RED", "x": 415, "area": 8200,
                             "distance": PASS_DISTANCE_MM - 50}]}
    approaching = machine_at(State.APPROACH_PILLAR)
    assert approaching.update(red, clear, driving, now=0)["state"] == "APPROACH_PILLAR"
    assert approaching.update(near_red, clear, driving,
                              now=0)["state"] == "PASS_PILLAR"

    # passing -> recentre once it is gone, never straight back to following
    passing = machine_at(State.PASS_PILLAR)
    assert passing.update(None, clear, driving, now=0)["state"] == "PASS_PILLAR"
    assert passing.update(None, clear, driving,
                          now=PILLAR_CLEAR_TIME_S)["state"] == "RECENTER"
    # losing sight of a pillar you are squeezing past is success, not a fault
    stalled_pass = machine_at(State.PASS_PILLAR)
    assert stalled_pass.update(red, clear, driving,
                               now=STATE_TIMEOUT_S[State.PASS_PILLAR])["state"] == "RECENTER"

    # seeing the pillar again during the approach resets the lost-sight wait
    approaching = machine_at(State.APPROACH_PILLAR)
    approaching.update(None, clear, driving, now=0)
    approaching.update(red, clear, driving, now=0.4)
    assert approaching.update(None, clear, driving, now=0.6)["state"] == "APPROACH_PILLAR"

    # --- recentring holds until the lane says we are straight again ---
    off_centre = {"front_distance": 900, "heading": 0.0, "speed": 40.0,
                  "left_distance": 300, "right_distance": 900}
    centred = {"front_distance": 900, "heading": 0.0, "speed": 40.0,
               "left_distance": 600, "right_distance": 600}
    recentring = machine_at(State.RECENTER)
    assert recentring.update(None, off_centre, driving, now=0)["state"] == "RECENTER"
    assert recentring.update(None, centred, driving, now=0)["state"] == "FOLLOW_COURSE"
    # and gives up rather than realigning forever
    recentring = machine_at(State.RECENTER)
    assert recentring.update(None, off_centre, driving,
                             now=STATE_TIMEOUT_S[State.RECENTER])["state"] == "FOLLOW_COURSE"
    # a pillar in view must not restart the pass while we are still recentring
    assert machine_at(State.RECENTER).update(
        red, off_centre, driving, now=0)["state"] == "RECENTER"

    # --- priority: a wall beats a pillar, because the table says so ---
    following = machine_at(State.FOLLOW_COURSE)
    result = following.update(red, close, driving, now=0)
    assert "RED_PILLAR" in result["events"] and "CORNER_DETECTED" in result["events"]
    assert result["state"] == "TURN_CORNER"

    # --- corners ---
    turning = machine_at(State.TURN_CORNER, _corner_start_heading=0.0)
    part_way = {"front_distance": 900, "heading": 40.0, "speed": 40.0}
    assert turning.update(None, part_way, driving, now=0)["state"] == "TURN_CORNER"
    done = {"front_distance": 900, "heading": float(CORNER_ANGLE_DEG), "speed": 40.0}
    assert turning.update(None, done, driving, now=0)["state"] == "RECENTER"
    assert turning.corners_turned == 1
    # turning the other way counts just the same
    turning = machine_at(State.TURN_CORNER, _corner_start_heading=0.0)
    back = {"front_distance": 900, "heading": 360.0 - CORNER_ANGLE_DEG, "speed": 40.0}
    assert turning.update(None, back, driving, now=0)["state"] == "RECENTER"

    # --- the run ends after twelve corners, not before ---
    nearly = machine_at(State.TURN_CORNER, _corner_start_heading=0.0,
                        corners_turned=CORNERS_PER_RUN - 1)
    assert nearly.update(None, done, driving, now=0)["state"] == "RECENTER"
    # twelve corners done: go and park, do not just stop in the middle of the mat
    assert nearly.update(None, clear, driving, now=0)["state"] == "SEARCH_PARKING"

    # --- parking, the last thing that happens in a run ---
    slot = {"pillars": [], "parking": {"markers": 2, "offset": 0.5}}
    lined_up = {"pillars": [], "parking": {"markers": 2, "offset": 0.0}}
    half = {"pillars": [], "parking": {"markers": 1, "offset": 0.0}}

    hunting = machine_at(State.SEARCH_PARKING)
    assert hunting.update(half, clear, driving, now=0)["state"] == "SEARCH_PARKING"
    assert hunting.update(slot, clear, driving, now=0)["state"] == "ALIGN_PARKING"

    aligning = machine_at(State.ALIGN_PARKING)
    assert aligning.update(slot, clear, driving, now=0)["state"] == "ALIGN_PARKING"
    assert aligning.update(lined_up, clear, driving, now=0)["state"] == "ENTER_PARKING"

    in_slot = dict(clear, front_distance=PARKING_STOP_MM - 20)
    entering = machine_at(State.ENTER_PARKING)
    assert entering.update(lined_up, clear, driving, now=0)["state"] == "ENTER_PARKING"
    assert entering.update(lined_up, in_slot, driving, now=0)["state"] == "FINISHED"

    # losing the slot mid-align sends us round for another look
    lost = machine_at(State.ALIGN_PARKING)
    assert lost.update(half, clear, driving,
                       now=STATE_TIMEOUT_S[State.ALIGN_PARKING])["state"] == "SEARCH_PARKING"
    # never finding it at all still ends the run safely rather than driving on
    never = machine_at(State.SEARCH_PARKING)
    assert never.update(None, clear, driving,
                        now=STATE_TIMEOUT_S[State.SEARCH_PARKING])["state"] == "FINISHED"
    # and entering stops even if the rear sensor never reports
    stubborn = machine_at(State.ENTER_PARKING)
    assert stubborn.update(lined_up, clear, driving,
                           now=STATE_TIMEOUT_S[State.ENTER_PARKING])["state"] == "FINISHED"

    parked = machine_at(State.FINISHED)
    for _ in range(3):      # terminal
        assert parked.update(red, clear, driving, now=99)["state"] == "FINISHED"

    # --- trouble ---
    stalled = machine_at(State.FOLLOW_COURSE)
    stopped = {"front_distance": 900, "heading": 0.0, "speed": 0.0}
    stalled.update(None, stopped, driving, now=0)
    assert stalled.update(None, stopped, driving,
                          now=STUCK_TIME_S)["state"] == "RECOVERY"

    slow_corner = machine_at(State.TURN_CORNER, _corner_start_heading=0.0)
    assert slow_corner.update(None, part_way, driving,
                              now=STATE_TIMEOUT_S[State.TURN_CORNER])["state"] == "RECOVERY"

    recovering = machine_at(State.RECOVERY)
    assert recovering.update(None, clear, driving, now=0)["state"] == "FOLLOW_COURSE"
    recovering = machine_at(State.RECOVERY, recovery_attempts=1)
    assert recovering.update(None, blocked, driving,
                             now=STATE_TIMEOUT_S[State.RECOVERY])["state"] == "FOLLOW_COURSE"
    # give up rather than thrash forever
    exhausted = machine_at(State.RECOVERY,
                           recovery_attempts=MAX_RECOVERY_ATTEMPTS + 1)
    assert exhausted.update(None, blocked, driving, now=0)["state"] == "FINISHED"

    # --- the report is always complete, even with no inputs at all ---
    report = StateMachine().update()
    assert set(report) == {"state", "reason", "mission", "events",
                           "speed", "steering"}, report
    assert report["state"] in {state.value for state in State}
    assert report["reason"]

    # --- the state machine restrains navigation, never invents a command ---
    assert set(STATE_SPEED_CAP) == set(State), "a state has no speed cap"
    assert set(STATE_SPEED_FLOOR) == set(State), "a state has no speed floor"

    # --- speed limits act on how fast, not on the signed number ---
    reversing = {"speed": -20, "steering": 0}
    # a cap slows a reverse without flipping it forwards
    assert apply_state(State.RECOVERY, reversing)[0] == -20
    assert apply_state(State.TURN_CORNER, {"speed": -50, "steering": 0})[0] == -35
    # a floor speeds a slow reverse up, still backwards
    assert apply_state(State.ENTER_PARKING, {"speed": -5, "steering": 0})[0] == -15
    # and a cap of zero still means stop, in either direction
    assert apply_state(State.FINISHED, reversing) == (0, 0)

    # a corner keeps moving even when navigation has given up, or it deadlocks
    assert apply_state(State.TURN_CORNER, {"speed": 0, "steering": 18})[0] > 0
    assert apply_state(State.TURN_CORNER, {"speed": 0, "steering": 18})[0] == \
        STATE_SPEED_FLOOR[State.TURN_CORNER]
    # ...but a stop still means stop everywhere it should
    assert apply_state(State.FOLLOW_COURSE, {"speed": 0, "steering": 0})[0] == 0
    assert apply_state(State.FINISHED, {"speed": 45, "steering": 18}) == (0, 0)
    # and the floor never lifts a speed above that state's own cap
    for state in State:
        floor = STATE_SPEED_FLOOR[state]
        cap = STATE_SPEED_CAP[state]
        if floor is not None and cap is not None:
            assert floor <= cap, state
    fast = {"speed": 45, "steering": 18}
    assert apply_state(State.FOLLOW_COURSE, fast) == (45, 18)   # untouched
    assert apply_state(State.APPROACH_PILLAR, fast) == (45, 18)  # avoiding is navigation
    assert apply_state(State.PASS_PILLAR, fast) == (45, 18)
    assert apply_state(State.RECENTER, fast) == (45, 18)
    # parking speed is navigation's job, the same way pillar avoidance is
    assert apply_state(State.ALIGN_PARKING, fast) == (45, 18)
    assert apply_state(State.RECOVERY, fast) == (30, 18)        # speed capped, steering kept
    assert apply_state(State.TURN_CORNER, fast) == (35, 18)
    assert apply_state(State.FINISHED, fast) == (0, 0)
    assert apply_state(State.WAIT_FOR_START, fast) == (0, 0)
    # a cap only ever lowers the speed
    slow = {"speed": 10, "steering": -5}
    assert apply_state(State.RECOVERY, slow) == (10, -5)
    assert apply_state(State.FOLLOW_COURSE, None) == (0, 0)     # missing data is safe

    # and the command comes back from update() ready to send
    running = StateMachine()
    running.start()
    running.state = State.RECOVERY
    result = running.update(None, {"front_distance": 100, "speed": 0.0}, fast, now=0)
    assert (result["speed"], result["steering"]) == (30, 18), result

    # --- missions gate which behaviours exist -------------------------------
    open_run = mission_manager.Mission.OPEN_CHALLENGE.value
    obstacle = mission_manager.Mission.OBSTACLE_CHALLENGE.value
    parking_only = mission_manager.Mission.PARKING.value

    # every mission is described, and every mission can start, recover and end
    assert set(MISSION_STATES) == set(MISSION_SUBSTITUTE)
    assert set(MISSION_STATES) == {m.value for m in mission_manager.Mission}
    for mission, states in MISSION_STATES.items():
        assert CORE_STATES <= states, mission

    # no transition may lead nowhere: every target is either in the mission or
    # has an explicit substitute. This is what stops a mission deadlocking.
    for mission in MISSION_STATES:
        for state in MISSION_STATES[mission]:
            for _, target in TRANSITIONS[state]:
                assert (target in MISSION_STATES[mission]
                        or target in MISSION_SUBSTITUTE[mission]), (mission, state, target)
    # and every substitute must itself be a state the mission has
    for mission, table in MISSION_SUBSTITUTE.items():
        for target in table.values():
            assert target is None or target in MISSION_STATES[mission], (mission, target)

    # OPEN: a pillar is scenery, so the machine stays on course
    open_machine = machine_at(State.FOLLOW_COURSE)
    open_machine.mission = open_run
    result = open_machine.update(red, clear, driving, now=0)
    assert "RED_PILLAR" in result["events"]          # still seen...
    assert result["state"] == "FOLLOW_COURSE"        # ...and correctly ignored
    assert result["mission"] == open_run

    # OPEN: three laps done means finished, not parking
    open_done = machine_at(State.FOLLOW_COURSE, corners_turned=CORNERS_PER_RUN)
    open_done.mission = open_run
    assert open_done.update(None, clear, driving, now=0)["state"] == "FINISHED"

    # OBSTACLE: the same two cases behave the way the round needs
    obstacle_machine = machine_at(State.FOLLOW_COURSE)
    obstacle_machine.mission = obstacle
    assert obstacle_machine.update(red, clear, driving,
                                   now=0)["state"] == "APPROACH_PILLAR"
    obstacle_done = machine_at(State.FOLLOW_COURSE, corners_turned=CORNERS_PER_RUN)
    obstacle_done.mission = obstacle
    assert obstacle_done.update(None, clear, driving,
                                now=0)["state"] == "SEARCH_PARKING"

    # PARKING: initialising goes straight to looking for the slot
    park_machine = StateMachine(parking_only)
    park_machine.start()
    park_machine.update(None, clear, driving, now=0)
    assert park_machine.update(None, clear, driving,
                               now=STATE_TIMEOUT_S[State.INITIALISE]
                               )["state"] == "SEARCH_PARKING"
    # and a corner is not its problem
    park_corner = machine_at(State.SEARCH_PARKING)
    park_corner.mission = parking_only
    assert park_corner.update(None, close, driving,
                              now=0)["state"] == "SEARCH_PARKING"

    # the mission is chosen by the Mission Manager unless one is given
    assert StateMachine().mission == mission_manager.current_mission()["mission"]

    print("selftest ok  %d states, %d events, %d transitions, %d missions"
          % (len(State), len(Event),
             sum(len(rows) for rows in TRANSITIONS.values()),
             len(MISSION_STATES)))


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        DEBUG = False
        selftest()
    else:
        simulate()
