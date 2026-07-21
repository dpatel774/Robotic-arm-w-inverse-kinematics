"""
main.py
-------
Pygame loop tying the project together:
    mouse (x, y)  -> reach + height  -> kinematics.solve_arm()
    scroll wheel  -> kinematics.update_base_angle()
    resulting (base, shoulder, elbow) -> arm_serial.send_angles()
    all three angles -> drawn on screen as a live arm preview

This file owns UI/orchestration only. It does not do IK math itself
(that's kinematics.py) and does not touch the serial protocol directly
(that's arm_serial.py) — see the project README for the full pipeline.

Runs fine with NO Arduino connected: if the serial connection fails,
it prints a warning and continues in preview-only mode, so you can
develop/test the visuals without hardware attached.
"""

import sys
import math
import pygame

from kinematics import solve_arm, update_base_angle, get_joint_positions

from arm_serial import ArmConnection

# ---- Config ----
PORT = "/dev/cu.wchusbserial10"
WINDOW_WIDTH = 900
WINDOW_HEIGHT = 600
FPS = 60

# Shoulder pivot's position on screen (pixels), sitting at the base
# of the window like a real desk-arm base. The visual mapping below
# is rotated 90 deg from kinematics.py's raw reach/height axes so the
# arm fans LEFT-RIGHT above this point (shoulder=0 -> left,
# shoulder=90 -> straight up, shoulder=180 -> right), matching how a
# desk lamp / desk arm actually looks from the front.
ORIGIN = (WINDOW_WIDTH // 2, WINDOW_HEIGHT - 60)
PX_PER_CM = 15  # scales cm (kinematics) to pixels (screen)

# Colors
COLOR_BG = (24, 24, 28)
COLOR_LINK = (210, 210, 220)
COLOR_JOINT = (255, 255, 255)
COLOR_TARGET_OK = (90, 200, 120)
COLOR_TARGET_BAD = (220, 90, 90)
COLOR_TEXT = (200, 200, 210)


def cm_to_px(reach_cm, height_cm):
    """Converts an (reach, height) point in cm to screen pixel coords.
    Mapping is rotated 90 deg from the raw kinematics axes: kinematics
    'height' drives screen x (left/right), kinematics 'reach' drives
    screen y (up from the base) -- this makes the arm fan left-right
    above its base instead of sweeping in a vertical side profile."""
    x = ORIGIN[0] + height_cm * PX_PER_CM
    y = ORIGIN[1] - reach_cm * PX_PER_CM
    return (x, y)


def px_to_cm(mouse_x, mouse_y):
    """Converts the mouse's screen pixel position to (reach, height) cm,
    inverse of the rotated mapping in cm_to_px.

    reach_cm is clamped to >= 0: mouse positions at or below the
    pivot's screen row correspond to "behind/below the shoulder",
    which isn't a real target for this arm. Left unclamped, small or
    negative reach_cm next to nonzero height_cm sends angle_to_target
    in solve_arm() toward +/-90 deg almost immediately, pinning the
    shoulder at 0 or 180 far sooner than the window's actual space
    would suggest."""
    height_cm = (mouse_x - ORIGIN[0]) / PX_PER_CM
    reach_cm = max(0.0, (ORIGIN[1] - mouse_y) / PX_PER_CM)
    return reach_cm, height_cm


def connect_to_arm():
    """Tries to connect to the Arduino. Returns None (preview-only mode)
    if that fails, rather than crashing the whole program."""
    try:
        arm = ArmConnection(PORT)
        print(f"[main] Connected to arm on {PORT}")
        return arm
    except ConnectionError as e:
        print(f"[main] {e}")
        print("[main] Continuing in PREVIEW-ONLY mode (no hardware connected).")
        return None


def draw_arm(screen, font, base_deg, shoulder_deg, elbow_deg, reachable):
    elbow_pos_cm, end_pos_cm = get_joint_positions(shoulder_deg, elbow_deg)

    origin_px = ORIGIN
    elbow_px = cm_to_px(*elbow_pos_cm)
    end_px = cm_to_px(*end_pos_cm)

    target_color = COLOR_TARGET_OK if reachable else COLOR_TARGET_BAD

    # Links
    pygame.draw.line(screen, COLOR_LINK, origin_px, elbow_px, 8)
    pygame.draw.line(screen, COLOR_LINK, elbow_px, end_px, 8)

    # Joints
    pygame.draw.circle(screen, COLOR_JOINT, origin_px, 10)
    pygame.draw.circle(screen, COLOR_JOINT, elbow_px, 8)
    pygame.draw.circle(screen, target_color, end_px, 8)

    # Reach envelope, for visual reference (max reach circle)
    max_reach_px = 22 * PX_PER_CM
    pygame.draw.circle(screen, (50, 50, 58), origin_px, max_reach_px, 1)

    # Readout text
    lines = [
        f"base:     {base_deg:>5.0f} deg",
        f"shoulder: {shoulder_deg:>5.0f} deg",
        f"elbow:    {elbow_deg:>5.0f} deg",
        f"reachable: {reachable}",
    ]
    for i, line in enumerate(lines):
        text_surface = font.render(line, True, COLOR_TEXT)
        screen.blit(text_surface, (20, 20 + i * 22))


def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("3-DOF Arm Control")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 16)

    arm = connect_to_arm()
    base_deg = 90.0  # start centered, matches Arduino's boot homing

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEWHEEL:
                base_deg = update_base_angle(base_deg, event.y)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

        # Mouse -> reach/height -> IK
        mouse_x, mouse_y = pygame.mouse.get_pos()
        reach_cm, height_cm = px_to_cm(mouse_x, mouse_y)
        shoulder_deg, elbow_deg, reachable = solve_arm(reach_cm, height_cm)

        # Send to hardware (throttled internally by ArmConnection;
        # safely skipped entirely in preview-only mode)
        if arm is not None:
            arm.send_angles(base_deg, shoulder_deg, elbow_deg)

        # Draw
        screen.fill(COLOR_BG)
        draw_arm(screen, font, base_deg, shoulder_deg, elbow_deg, reachable)
        pygame.display.flip()
        clock.tick(FPS)

    if arm is not None:
        arm.close()
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()