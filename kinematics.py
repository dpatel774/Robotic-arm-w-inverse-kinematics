"""
kinematics.py
-------------
Pure inverse-kinematics math for the 3-DOF arm. No serial, no Pygame —
this module only does geometry so it can be tested and debugged in
isolation from hardware.

Input model:
    - Mouse (x, y) on screen -> mapped by main.py into (reach, height)
      in centimeters, in the arm's vertical operating plane.
    - Scroll wheel -> base rotation, handled separately as a simple
      accumulator (no law-of-cosines needed, it's a direct angle).

Arm geometry (locked):
    L1 = 10 cm  (upper arm: shoulder -> elbow)
    L2 = 12 cm  (forearm:   elbow -> wrist/end effector)
    Max reach   = 22 cm (fully extended)
    Min reach   = 2 cm  (fully folded, |L1 - L2|)

Servo angle convention (SERVO angles, 0-180, sent to Arduino):
    - Base:     0-180, straightforward rotation, no conversion needed.
    - Shoulder: 90 = pointing straight up/vertical reference,
                increases as the arm reaches further forward.
    - Elbow:    180 = fully extended (straight line with upper arm),
                decreases as the elbow folds.
    These conventions are a starting assumption based on typical servo
    mounting and WILL likely need a small offset/calibration once
    tested on the physical arm (see moveJoint() note in Arduino sketch
    re: SERVO_MIN/SERVO_MAX per-servo tuning). If the arm visually
    reaches the wrong direction, adjust SHOULDER_OFFSET_DEG /
    ELBOW_FULL_EXTEND_DEG below rather than the math itself.
"""

import math

# ---- Arm geometry ----
L1 = 10.0  # cm, upper arm length
L2 = 12.0  # cm, forearm length
MAX_REACH = L1 + L2
MIN_REACH = abs(L1 - L2)

# ---- Servo angle convention offsets (tune during physical testing) ----
SHOULDER_OFFSET_DEG = 90.0   # servo angle when shoulder geometric angle = 0

# Elbow horn is mounted flipped (physically rotated 180 deg) so the
# joint bends in the opposite rotational direction from the original
# build. That makes servo 0 (not 180) the "fully extended" position.
ELBOW_FULL_EXTEND_DEG = 0.0

# Elbow configuration: +1 = "elbow down", -1 = "elbow up".
# With the horn flipped, -1 is now the config that uses the servo's
# full 0-180 range across the whole 2-22cm reach (mirrored from the
# original build, where +1 was the one that fit).
ELBOW_BEND_SIGN = -1

# ---- Servo safety range ----
SERVO_ANGLE_MIN = 0
SERVO_ANGLE_MAX = 180


def clamp(value, low, high):
    return max(low, min(high, value))


def solve_arm(reach_cm, height_cm):
    """
    Solve shoulder/elbow servo angles for a target point in the arm's
    vertical plane, using the elbow-down configuration.

    Args:
        reach_cm: forward horizontal distance from the shoulder pivot (cm)
        height_cm: vertical distance from the shoulder pivot (cm)
                   (positive = above the shoulder, negative = below)

    Returns:
        (shoulder_servo_deg, elbow_servo_deg, reachable)
        reachable is False if the target was outside [MIN_REACH, MAX_REACH]
        and had to be clamped onto the nearest valid point.
    """
    reachable = True

    # Distance from shoulder pivot to target
    d = math.hypot(reach_cm, height_cm)

    if d > MAX_REACH:
        # Clamp target onto the max-reach circle, same direction
        scale = MAX_REACH / d
        reach_cm *= scale
        height_cm *= scale
        d = MAX_REACH
        reachable = False
    elif d < MIN_REACH:
        if d == 0:
            # Degenerate case: target is exactly at the shoulder pivot.
            # Push it out to the minimum reachable distance along +reach.
            reach_cm, height_cm = MIN_REACH, 0.0
        else:
            scale = MIN_REACH / d
            reach_cm *= scale
            height_cm *= scale
        d = MIN_REACH
        reachable = False

    # Law of cosines gives the INTERIOR triangle angle at the elbow
    # (180 deg when fully extended, 0 deg when fully folded). We need
    # theta2, the elbow BEND angle instead (0 deg when fully extended,
    # up to 180 deg when fully folded) — supplementary to the interior
    # angle — then apply ELBOW_BEND_SIGN to pick which side the elbow
    # bulges toward (down = +1, up = -1).
    cos_elbow_interior = (L1 ** 2 + L2 ** 2 - d ** 2) / (2 * L1 * L2)
    cos_elbow_interior = clamp(cos_elbow_interior, -1.0, 1.0)  # guard fp error
    elbow_interior_rad = math.acos(cos_elbow_interior)
    theta2_mag_rad = math.pi - elbow_interior_rad  # bend magnitude, 0 = straight
    theta2_rad = ELBOW_BEND_SIGN * theta2_mag_rad

    # Shoulder angle: angle to target minus the "shoulder offset" caused
    # by the elbow bending
    angle_to_target_rad = math.atan2(height_cm, reach_cm)
    offset_rad = math.atan2(
        L2 * math.sin(theta2_rad),
        L1 + L2 * math.cos(theta2_rad)
    )
    shoulder_geom_rad = angle_to_target_rad - offset_rad

    # Convert geometry -> servo angle convention
    shoulder_servo_deg = SHOULDER_OFFSET_DEG + math.degrees(shoulder_geom_rad)
    elbow_servo_deg_raw = ELBOW_FULL_EXTEND_DEG - math.degrees(theta2_rad)

    # Both shoulder and elbow can compute raw angles outside the
    # servo's real 0-180 range (e.g. targets behind the shoulder pivot,
    # or elbow configs the physical joint can't produce) — flag those
    # as unreachable rather than silently clamping into something that
    # looks plausible on screen but isn't physically achievable.
    if shoulder_servo_deg < SERVO_ANGLE_MIN or shoulder_servo_deg > SERVO_ANGLE_MAX:
        reachable = False
    if elbow_servo_deg_raw < SERVO_ANGLE_MIN or elbow_servo_deg_raw > SERVO_ANGLE_MAX:
        reachable = False

    shoulder_servo_deg = clamp(round(shoulder_servo_deg), SERVO_ANGLE_MIN, SERVO_ANGLE_MAX)
    elbow_servo_deg = clamp(round(elbow_servo_deg_raw), SERVO_ANGLE_MIN, SERVO_ANGLE_MAX)

    return shoulder_servo_deg, elbow_servo_deg, reachable


def get_joint_positions(shoulder_servo_deg, elbow_servo_deg):
    """
    Forward kinematics for drawing: given servo angles, returns the
    (reach, height) position of the elbow and the end effector, in cm,
    relative to the shoulder pivot. Used by main.py to draw the arm
    preview — kept here rather than duplicated in main.py so the
    drawing always matches the same geometry solve_arm() uses.

    Returns:
        (elbow_pos, end_pos), each an (reach_cm, height_cm) tuple.
    """
    theta1_rad = math.radians(shoulder_servo_deg - SHOULDER_OFFSET_DEG)
    theta2_rad = math.radians(ELBOW_FULL_EXTEND_DEG - elbow_servo_deg)

    elbow_reach = L1 * math.cos(theta1_rad)
    elbow_height = L1 * math.sin(theta1_rad)

    end_reach = elbow_reach + L2 * math.cos(theta1_rad + theta2_rad)
    end_height = elbow_height + L2 * math.sin(theta1_rad + theta2_rad)

    return (elbow_reach, elbow_height), (end_reach, end_height)


def update_base_angle(current_base_deg, scroll_delta, degrees_per_step=3):
    """
    Accumulator for scroll-wheel base rotation control.

    Args:
        current_base_deg: current base servo angle (0-180)
        scroll_delta: raw scroll event value from Pygame
                      (positive = scroll up, negative = scroll down)
        degrees_per_step: how many degrees each scroll "click" moves the base

    Returns:
        new_base_deg, clamped to [0, 180]
    """
    new_base_deg = current_base_deg + (scroll_delta * degrees_per_step)
    return clamp(new_base_deg, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX)


if __name__ == "__main__":
    # Quick sanity checks — run `python3 kinematics.py` to verify the
    # math independently of Pygame/serial.
    test_points = [
        (15, 0),    # straight out, mid-height
        (0, 22),    # fully extended straight up (edge of max reach)
        (22, 0),    # fully extended straight out (max reach)
        (1, 1),     # very close to shoulder (should clamp, unreachable)
        (30, 10),   # way past max reach (should clamp, unreachable)
    ]

    print(f"{'reach':>6} {'height':>7} {'shoulder':>9} {'elbow':>6} {'reachable':>10}")
    for reach, height in test_points:
        s, e, ok = solve_arm(reach, height)
        print(f"{reach:6.1f} {height:7.1f} {s:9d} {e:6d} {str(ok):>10}")

    print()
    base = 90
    for delta in [1, 1, -1, -1, -1, 50, -50]:
        base = update_base_angle(base, delta)
        print(f"scroll {delta:+d} -> base = {base}")