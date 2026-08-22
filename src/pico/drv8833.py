"""Drive motor control through a DRV8833 on the Pico 2 W.  MicroPython.

Reads "speed,steering" lines from the Pi over UART and drives one motor.
The steering value is parsed and ignored - this module only drives.

Speed is signed. Positive drives forward, negative reverses, zero coasts.

On the Pico:  run this file from Thonny.
On a laptop:  python3 -c "import drv8833; drv8833.selftest()"   (maths only)
"""

try:
    from machine import PWM, Pin, UART  # type: ignore
except ImportError:          # laptop: the conversion below is still testable
    PWM = Pin = UART = None

# --- Motor ------------------------------------------------------------------
AIN1_PIN = 8                # physical pin 11. PWM here = forward
AIN2_PIN = 9                # physical pin 12. Held low for forward

FREQ = 20_000               # 20 kHz: above hearing, so the motor does not whine

# Drop this to 40 or so for the first bench test, then raise it once the robot
# behaves. Applies to both directions.
MAX_SPEED = 100

# --- UART (same link as uart_test_pi.py) ------------------------------------
BAUD = 115200


def speed_to_duty(speed):
    """Signed speed -100 to 100 -> (clamped speed, PWM duty magnitude).

    The duty comes back positive whichever way we are going, because on a
    DRV8833 the direction is decided by WHICH of the two inputs carries the PWM,
    not by the value on it. The sign is preserved so the caller knows which
    input to drive.
    """
    speed = max(-MAX_SPEED, min(MAX_SPEED, speed))
    return speed, int(abs(speed) * 65535 / 100)


def main():
    # Both inputs are PWM now. Which one carries the signal is the direction;
    # the other is held at zero. GP8 and GP9 are the two halves of the same PWM
    # slice, so they necessarily share a frequency, which suits us because both
    # want the same 20 kHz.
    ain1 = PWM(Pin(AIN1_PIN))
    ain1.freq(FREQ)
    ain2 = PWM(Pin(AIN2_PIN))
    ain2.freq(FREQ)
    ain1.duty_u16(0)
    ain2.duty_u16(0)
    uart = UART(0, baudrate=BAUD, tx=Pin(0), rx=Pin(1))
    print("Motor ready. Max speed:", MAX_SPEED)

    buffer = b""
    while True:
        if uart.any():
            buffer += uart.read()

            # A read can hold half a line or several lines, so drain whole ones.
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)

                try:
                    values = [int(v) for v in line.decode().strip().split(",")]
                    speed = values[0]            # values[1] is steering, ignored here
                except (ValueError, IndexError, UnicodeError):
                    print("Ignored:", line)
                    continue

                speed, duty = speed_to_duty(speed)
                # Zero the opposite input BEFORE energising the other one, so
                # the bridge is never driven both ways at the same instant.
                if speed >= 0:
                    ain2.duty_u16(0)
                    ain1.duty_u16(duty)
                else:
                    ain1.duty_u16(0)
                    ain2.duty_u16(duty)

                print("Received Speed:", speed)
                print("PWM Duty:", duty)
                print("Motor Stopped" if speed == 0 else
                      "Motor Running Forward" if speed > 0 else
                      "Motor Running Reverse")

        if len(buffer) > 200:
            buffer = b""


def selftest():
    assert speed_to_duty(0) == (0, 0)
    assert speed_to_duty(100) == (100, 65535)

    # reverse keeps its sign, and the duty is the magnitude either way
    assert speed_to_duty(-100) == (-100, 65535)
    assert speed_to_duty(-30)[0] == -30
    assert speed_to_duty(-30)[1] == speed_to_duty(30)[1]
    assert speed_to_duty(-1)[1] > 0          # a small reverse is not a stop

    # out of range clamps in both directions instead of overflowing the register
    assert speed_to_duty(255) == speed_to_duty(MAX_SPEED)
    assert speed_to_duty(-255) == speed_to_duty(-MAX_SPEED)
    assert speed_to_duty(MAX_SPEED)[1] <= 65535
    assert speed_to_duty(-MAX_SPEED)[1] <= 65535

    # faster in always means a bigger duty out, whichever way we are going
    for direction in (1, -1):
        duties = [speed_to_duty(direction * s)[1] for s in range(0, 101, 10)]
        assert duties == sorted(duties) and len(set(duties)) == len(duties)

    print("selftest ok  fwd 30%% -> %d   rev 30%% -> %d   full -> %d"
          % (speed_to_duty(30)[1], speed_to_duty(-30)[1], speed_to_duty(100)[1]))


if __name__ == "__main__":
    main()
