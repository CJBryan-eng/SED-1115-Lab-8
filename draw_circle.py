"""Lab 8: draw a circle with inverse kinematics, with or without calibration.

Run once with USE_CALIBRATION = False and once with True (on a fresh spot
on the paper) and compare the two circles.
"""
from machine import Pin, PWM
from time import sleep
import math
from servo_translator import translate
from kinematics_calibration import (inverse_kinematics, calibration_setup,
                                    send_compensated_angle)

USE_CALIBRATION = False
CALIBRATION_FILE = "calibration_data_X.txt"   # change X to your jig ID

CENTER_X = 108      # mm, middle of a letter-size sheet
CENTER_Y = 139.5
RADIUS = 40
POINTS = 36         # one point every 10 degrees
PEN_UP = 0
PEN_DOWN = 30

shoulder = PWM(Pin(0))
elbow = PWM(Pin(1))
wrist = PWM(Pin(2))
for servo in (shoulder, elbow, wrist):
    servo.freq(50)


def disable_servos():
    shoulder.duty_u16(0)
    elbow.duty_u16(0)
    wrist.duty_u16(0)


def move_to(x, y, table):
    angle_a, angle_b = inverse_kinematics(x, y)
    if table is None:
        shoulder.duty_u16(translate(angle_a))
        elbow.duty_u16(translate(angle_b))
    else:
        send_compensated_angle(shoulder, angle_a, table["shoulder"])
        send_compensated_angle(elbow, angle_b, table["elbow"])
    sleep(0.15)


def main():
    table = calibration_setup(CALIBRATION_FILE) if USE_CALIBRATION else None
    try:
        wrist.duty_u16(translate(PEN_UP))
        move_to(CENTER_X + RADIUS, CENTER_Y, table)
        sleep(0.5)
        wrist.duty_u16(translate(PEN_DOWN))
        sleep(0.3)
        for i in range(POINTS + 1):
            angle = 2 * math.pi * i / POINTS
            move_to(CENTER_X + RADIUS * math.cos(angle),
                    CENTER_Y + RADIUS * math.sin(angle), table)
        wrist.duty_u16(translate(PEN_UP))
        sleep(0.3)
    finally:
        disable_servos()


main()
