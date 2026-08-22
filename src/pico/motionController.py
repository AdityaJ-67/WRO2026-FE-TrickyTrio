"""Motion Controller: one call to set how fast and which way the robot goes.

    move(speed, steering)   speed -100 to 100, steering -30 to +30 degrees
    stop()                  motor off, wheels centred

Positive speed drives forward, negative reverses. Reverse is needed for
parallel parking, for backing out of a bad park, and for recovering when the
robot wedges itself against a wall.

Coordination only. The clamping and the duty-cycle maths already live in
drv8833.py and servo.py, and this module calls them rather than repeating them.
No PID, no acceleration ramps, no autonomy.

On the Pico:  mpremote run motionController.py
On a laptop:  python3 -c "import motionController; motionController.selftest()"
"""

try:
    from machine import PWM, Pin        # type: ignore
    from time import sleep_ms           # type: ignore
except ImportError:                     # laptop: the coordination is still testable
    PWM = Pin = None

    def sleep_ms(ms):
        pass

import drv8833
import servo

# Hardware handles, filled in by setup().
motor_forward = None        # DRV8833 AIN1
motor_reverse = None        # DRV8833 AIN2
steering_pwm = None

# Direction of the last command, so a reversal can be softened. Slamming a
# turning motor into the opposite direction draws a large current spike, and
# the 5 V rail is the fragile part of this robot.
_last_direction = 0

STEP_MS = 1500      # how long the test loop holds each command


def setup():
    """Create the three PWM outputs. Call once before move()."""
    global motor_forward, motor_reverse, steering_pwm

    # On a DRV8833 the direction is decided by which input carries the PWM, so
    # both are PWM outputs and the idle one is held at zero.
    motor_forward = PWM(Pin(drv8833.AIN1_PIN))
    motor_forward.freq(drv8833.FREQ)
    motor_reverse = PWM(Pin(drv8833.AIN2_PIN))
    motor_reverse.freq(drv8833.FREQ)

    steering_pwm = PWM(Pin(servo.SERVO_PIN))
    steering_pwm.freq(servo.FREQ)

    stop()
    print("Motion ready. Speed 0-%d, steering +/-%d deg"
          % (drv8833.MAX_SPEED, servo.MAX_STEER))


def move(speed, steering):
    """Drive at speed (-100 to 100) while steering (-30 to +30 degrees).

    Positive speed is forward, negative is reverse. Both values are clamped by
    the modules that own them, and the clamped values are returned so the caller
    can see what actually got used.
    """
    global _last_direction

    speed, motor_duty = drv8833.speed_to_duty(speed)
    angle, servo_duty = servo.steering_to_duty(steering)
    direction = 0 if speed == 0 else (1 if speed > 0 else -1)

    # Changing direction while still moving is the one case that draws a large
    # current spike, so pass through a coast first. One loop of delay costs
    # nothing next to a brownout that reboots the Pi.
    if direction and _last_direction and direction != _last_direction:
        motor_forward.duty_u16(0)
        motor_reverse.duty_u16(0)

    # Zero the idle input BEFORE energising the other, so the bridge is never
    # driven both ways at the same instant.
    if direction >= 0:
        motor_reverse.duty_u16(0)
        motor_forward.duty_u16(motor_duty)
    else:
        motor_forward.duty_u16(0)
        motor_reverse.duty_u16(motor_duty)

    steering_pwm.duty_u16(servo_duty)
    _last_direction = direction
    return speed, angle


def stop():
    """Motor off, wheels straight. Coasts rather than brakes."""
    return move(0, 0)


def main():
    setup()

    for speed, steering in ((30, 0), (50, -20), (50, 20), (0, 0), (-30, 0)):
        actual_speed, actual_angle = move(speed, steering)
        print("move(%d, %d) -> speed %d, steering %d deg"
              % (speed, steering, actual_speed, actual_angle))
        sleep_ms(STEP_MS)

    stop()
    print("stop() -> speed 0, steering 0 deg")


def selftest():
    global motor_forward, motor_reverse, steering_pwm, _last_direction

    class _FakePWM:             # test double, not part of the module
        def __init__(self):
            self.duty = None

        def duty_u16(self, value):
            self.duty = value

    motor_forward, motor_reverse, steering_pwm = _FakePWM(), _FakePWM(), _FakePWM()
    _last_direction = 0

    # in-range values pass straight through
    assert move(30, 0) == (30, 0)
    assert move(50, -20) == (50, -20)
    assert move(50, 20) == (50, 20)

    # forward drives AIN1 and leaves AIN2 idle
    move(50, 20)
    assert motor_forward.duty == drv8833.speed_to_duty(50)[1]
    assert motor_reverse.duty == 0
    assert steering_pwm.duty == servo.steering_to_duty(20)[1]

    # --- reverse drives the other input, and only the other input ---
    stop()
    assert move(-40, 0) == (-40, 0)
    assert motor_reverse.duty == drv8833.speed_to_duty(40)[1]
    assert motor_forward.duty == 0

    # steering still works while reversing
    assert move(-40, 20)[1] == 20
    assert steering_pwm.duty == servo.steering_to_duty(20)[1]

    # the two inputs are never both driven at once, whatever we ask for
    for speed in (-100, -30, 0, 30, 100):
        move(speed, 0)
        assert motor_forward.duty == 0 or motor_reverse.duty == 0, speed

    # clamping is delegated, so the limits are whatever those modules say
    assert move(999, 0)[0] == drv8833.MAX_SPEED
    assert move(-999, 0)[0] == -drv8833.MAX_SPEED
    assert move(0, 90)[1] == servo.MAX_STEER
    assert move(0, -90)[1] == -servo.MAX_STEER

    # stop is exactly move(0, 0) - both inputs off and wheels centred
    assert stop() == (0, 0)
    assert motor_forward.duty == 0 and motor_reverse.duty == 0
    assert steering_pwm.duty == servo.steering_to_duty(0)[1]

    # a hard-over command followed by stop must leave nothing latched on
    move(100, 30)
    stop()
    assert motor_forward.duty == 0 and motor_reverse.duty == 0

    # reversing straight from full forward passes through a coast first
    move(100, 0)
    move(-100, 0)
    assert motor_forward.duty == 0
    assert motor_reverse.duty == drv8833.speed_to_duty(100)[1]

    print("selftest ok  fwd 50 -> AIN1 %d   rev 50 -> AIN2 %d   servo %d"
          % (drv8833.speed_to_duty(50)[1], drv8833.speed_to_duty(-50)[1],
             servo.steering_to_duty(-20)[1]))


if __name__ == "__main__":
    main()
