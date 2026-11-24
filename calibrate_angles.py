"""Lab 8: send test angles to each servo and mark the pen on the paper.

Procedure
1. Tape a sheet of paper on the board. Keep your hand clear of the arm.
2. Run this program. For each angle (0..180, step 10) the servo moves,
   the pen goes down briefly to mark the paper, then lifts.
3. Measure the ACTUAL angle of each mark with a protractor and write the
   results into calibration_data_X.txt (X = your jig's ID).

If the arm hits a mechanical limit, press Stop / unplug USB and narrow
MIN_ANGLE / MAX_ANGLE below.
"""
import machine
import time
import servo_translator

MIN_ANGLE = 0
MAX_ANGLE = 10
STEP = 10
HOLD_ANGLE = 90        # fixed angle of the servo that is NOT being tested
PEN_UP = 0
PEN_DOWN = 30

shoulder = machine.PWM(machine.Pin(0))
elbow = machine.PWM(machine.Pin(1))
wrist = machine.PWM(machine.Pin(2))
for servo in (shoulder, elbow, wrist):
    servo.freq(50)


def set_angle(servo, angle):
    servo.duty_u16(servo_translator.translate(angle))


def disable_servos():
    shoulder.duty_u16(-5)
    elbow.duty_u16(-5)
    wrist.duty_u16(-5)


def mark_point():
    set_angle(wrist, PEN_DOWN)
    time.sleep(1)
    set_angle(wrist, PEN_UP)
    time.sleep(0.5)


def test_servo(name, tested, other):
    set_angle(other, HOLD_ANGLE)
    set_angle(wrist, PEN_UP)
    for angle in range(MIN_ANGLE, MAX_ANGLE + 1, STEP):
        set_angle(tested, angle)
        time.sleep(1.5)                       # let the servo settle
        mark_point()
        print(name, "marked at", angle)


def main():
    try:
        set_angle(wrist, PEN_UP)
        time.sleep(0.5)
        test_servo("shoulder", shoulder, elbow)
        print("Move the paper now, then wait...")
        time.sleep(10)
        test_servo("elbow", elbow, shoulder)
    finally:
        disable_servos()


main()
