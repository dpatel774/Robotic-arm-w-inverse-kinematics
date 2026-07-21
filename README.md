# 3-DOF Desktop Robotic Arm

A servo-driven robotic arm built from scratch — CAD, 3D-printed structure, inverse kinematics, and real-time control — as an end-to-end hardware/software product development exercise.

![status](https://img.shields.io/badge/status-in--progress-yellow)

**[Live demo & full writeup →](https://dpatel774.github.io/Robotic-arm-w-inverse-kinematics/)**

---

## Overview

This project is a 3-degree-of-freedom robotic arm controlled in real time via mouse and scroll-wheel input. It was built to demonstrate the complete design-to-build cycle: mechanical design in CAD, 3D-printed structure, custom inverse kinematics math, and a full control pipeline from software down to physical hardware.

- **Reach:** 2–22 cm (10 cm upper arm + 12 cm forearm)
- **Control:** live mouse-driven end-effector targeting + scroll-wheel base rotation
- **Pipeline:** Python (IK + UI) → Serial → Arduino (hardware translator) → PCA9685 PWM driver → servos

## Demo

*[Video placeholder — embed once recorded]*

## Hardware

| Component | Detail |
|---|---|
| Microcontroller | ELEGOO Uno R3 (CH340 USB) |
| PWM driver | PCA9685, 16-channel, I2C address `0x40` |
| Servos | 3x MG995 (base, shoulder, elbow) |
| Claw (v2) | SG90, reserved on channel 3 |
| Structure | 3D-printed (SolidWorks CAD), printed via CAEN facilities |

Full bill of materials: [`Robot Arm BOM.xlsx`](./Robot%20Arm%20BOM.xlsx)

## Software Architecture

The system is split into clearly separated layers, each with a single responsibility:

```
main.py            Pygame loop — UI, input handling, live arm visualization
kinematics.py       Pure IK math (no hardware, no UI) — testable in isolation
arm_serial.py        Serial transport layer — Python <-> Arduino handshake/protocol
arm_controller.ino    Hardware translator — receives angles, drives PCA9685/servos
```

**Design principle:** each layer only knows about the layer directly below it. `main.py` never touches serial directly, `kinematics.py` never touches hardware, and the Arduino sketch does no math — it just relays angles. This kept debugging isolated: a physical servo inversion, for example, was fixed at the firmware layer rather than leaking into the Python math or UI.

### Control flow

```
Mouse (x, y) → reach/height (cm) → solve_arm() → (shoulder, elbow) servo angles
Scroll wheel → base angle accumulator
   ↓
arm_serial.send_angles() → "base,shoulder,elbow\n" over USB (115200 baud)
   ↓
arm_controller.ino → PCA9685 → 3x MG995 servos
```

The Python side also runs fine with no Arduino connected (preview-only mode), so the IK and visualization can be developed and tested independently of the physical hardware.

## Inverse Kinematics

The arm uses a standard 2-link planar IK solution (law of cosines) to convert a target (reach, height) point into shoulder/elbow angles, with:

- Reachability clamping for targets outside the [2, 22] cm working envelope
- An "elbow-up" configuration, with a sign convention (`ELBOW_BEND_SIGN`) chosen to match the physical horn mounting
- Per-joint offset constants (`SHOULDER_OFFSET_DEG`, `ELBOW_FULL_EXTEND_DEG`) that map servo angle conventions to physical geometry — kept separate from the geometry math itself so hardware calibration never requires touching the math

See [`kinematics.py`](./kinematics.py) for the full implementation and inline derivation notes.

## Key Engineering Decisions

- **Firmware-level fixes over math-level workarounds** — a physical servo inversion was corrected in the Arduino sketch (per-channel angle flip), not by adjusting the Python IK math, keeping the geometry solve hardware-agnostic.
- **Open-loop calibration, not closed-loop control** — the system has no position feedback sensors; joint offsets are static calibration constants tuned against known physical positions, not PID-controlled.
- **Coupon-first 3D printing** — with per-gram print costs, critical fit features (like servo horn pockets) were validated on small test prints before committing to full-link reprints.

## Status & Roadmap

- [x] Full control loop working end-to-end (Python → serial → Arduino → PCA9685 → servos)
- [x] Shoulder inversion and elbow-up configuration resolved
- [ ] Servo pulse range (`SERVO_MIN`/`SERVO_MAX`) calibration to actual MG995 spec
- [ ] Elbow mount redesign (single continuous body, matching shoulder approach)
- [ ] Final link prints
- [ ] Demo video
- [ ] FEA: static stress analysis on highest-load part under ~2 kg payload
- [ ] v2: SG90 claw integration (channel 3, already wired)

## Repo Structure

```
kinematics.py        Inverse kinematics math
arm_serial.py         Serial transport layer
main.py                Pygame control loop / visualization
arm_controller.ino      Arduino firmware
Robot Arm BOM.xlsx        Bill of materials
docs/                       GitHub Pages source
```

## Running It

```bash
pip install pygame pyserial
python3 main.py
```

Runs in preview-only mode with no Arduino connected. To drive real hardware, update `PORT` in `main.py` to match your serial device and flash `arm_controller.ino` to the board first.

---

Built by [Dilan Patel](https://github.com/dpatel774) — University of Michigan.
