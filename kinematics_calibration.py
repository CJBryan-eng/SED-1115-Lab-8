"""Lab 8: inverse kinematics + calibration helpers.

Contains: inverse_kinematics, calibration_setup, send_compensated_angle
(plus interpolate_error, used by send_compensated_angle).
"""
import math
import servo_translator

# Arm geometry (mm), from the lab diagram
LA = 155            # upper arm length
LB = 155            # forearm length
AX = -50            # shoulder is 50 mm left of the paper
AY = 139.5          # shoulder is vertically centred on the paper

# Offsets from equations (8) and (9) -- tune these for your jig
SERVO_A_OFFSET = 75
SERVO_B_BASE = 150


def inverse_kinematics(cx, cy):
    """Pen position (cx, cy) in mm -> (servoA, servoB) angles in degrees.

    servoA = shoulder servo, servoB = elbow servo.
    Raises ValueError if the point is out of reach.
    """
    ac = math.sqrt((AX - cx) ** 2 + (AY - cy) ** 2)              # (1)
    abase_c = math.sqrt((AX - cx) ** 2 + cy ** 2)                # (2)

    if ac > LA + LB or ac == 0:
        raise ValueError("point out of reach")

    bac = math.acos((LA ** 2 + ac ** 2 - LB ** 2) / (2 * LA * ac))        # (3)
    acb = math.asin(LA * math.sin(bac) / LB)                              # (4)
    yac = math.acos((AY ** 2 + ac ** 2 - abase_c ** 2) / (2 * AY * ac))   # (5)

    alpha = math.degrees(bac + yac)    # (6)
    beta = math.degrees(bac + acb)     # (7)

    servo_a = alpha - SERVO_A_OFFSET   # (8)
    servo_b = SERVO_B_BASE - beta      # (9)
    return servo_a, servo_b


def calibration_setup(filename):
    """Read a calibration file and return the table of corrections.

    File format (see calibration_data_X.txt): lines of
        desired, shoulder_actual, elbow_actual
    Blank lines and lines starting with # are ignored.

    Returns {"shoulder": [(desired, correction), ...],
             "elbow":    [(desired, correction), ...]}
    where correction = desired - actual, i.e. the amount to ADD to a desired
    angle so that the servo really ends up at that angle.
    """
    table = {"shoulder": [], "elbow": []}
    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if line == "" or line.startswith("#"):
                continue
            parts = line.split(",")
            desired = float(parts[0])
            shoulder_actual = float(parts[1])
            elbow_actual = float(parts[2])
            table["shoulder"].append((desired, desired - shoulder_actual))
            table["elbow"].append((desired, desired - elbow_actual))
    table["shoulder"].sort()
    table["elbow"].sort()
    return table


def interpolate_error(servo_table, angle):
    """Linear interpolation of the correction for any angle.

    servo_table is a sorted list of (desired, correction) pairs.
    Angles outside the table use the nearest end value.
    """
    if angle <= servo_table[0][0]:
        return servo_table[0][1]
    if angle >= servo_table[-1][0]:
        return servo_table[-1][1]
    for i in range(len(servo_table) - 1):
        a0, e0 = servo_table[i]
        a1, e1 = servo_table[i + 1]
        if a0 <= angle <= a1:
            return e0 + (e1 - e0) * (angle - a0) / (a1 - a0)


def send_compensated_angle(servo, desired_angle, servo_table):
    """Add the interpolated correction to the desired angle, then send it.

    servo is a PWM object; servo_table is table["shoulder"] or table["elbow"].
    Returns the corrected angle that was sent.
    """
    corrected = desired_angle + interpolate_error(servo_table, desired_angle)
    servo.duty_u16(servo_translator.translate(corrected))
    return corrected
