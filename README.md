# AutonomousMicroBot

Autonomous differential-drive robot for the [Science Olympiad Robot Tour](https://scioly.org/wiki/index.php/Robot_Tour) event. Built on the **Pololu 3Pi+ 32U4** with closed-loop PID control, encoder feedback, and IMU heading correction.

## Hardware

| Component | Details |
|---|---|
| Platform | Pololu 3Pi+ 32U4 (ATmega32U4) |
| Motors | Dual micro gearmotors w/ encoders |
| IMU | LSM6DS33 gyro + accelerometer |
| Language | C++ / Arduino |

## Features

- **Swerve turns** — arc-based turning via differential wheel velocities: `V_inner/V_outer = (R-r)/(R+r)`
- **Pivot turns** — tight point rotations with one wheel stationary
- **Cascaded PID control** — per-wheel velocity tracking + IMU angular rate correction
- **Gyro filtering** — exponential low-pass filter with auto bias calibration at startup
- **Custom time-control system** — timed motion segments, trapezoidal speed profiling, and global run-time targeting (see below)

## Time-Control System

The robot uses a three-layer time-control approach to maximize scoring:

1. **Timed segments** — each motion primitive (straight or turn) executes for a computed duration rather than relying solely on encoder distance, improving consistency across surface conditions
2. **Speed profiling** — trapezoidal acceleration/deceleration ramps prevent wheel slip at segment starts and stops, reducing overshoot and encoder drift
3. **Global run timer** — overall velocity setpoints are scaled so the robot completes the full course within a target finish time, letting you trade speed for accuracy or vice versa

## Repo Structure

```
├── swerve_turning/      # Arc-based turning implementation
├── pivot_turning/       # Point turning implementation
├── UT25/                # UT Invitational 2025 course
├── ATX_Regionals/       # Austin Regionals course
├── Mock Trial/          # Practice configs
├── Test-Offs/           # Team qualifier configs
```

## Quick Start

```bash
git clone https://github.com/jaysahni/AutonomousMicroBot.git
```

1. Install the [Pololu 3Pi+ 32U4 library](https://github.com/pololu/pololu-3pi-plus-32u4-arduino-library) via Arduino Library Manager
2. Open a config folder (e.g. `swerve_turning/`) in Arduino IDE
3. Board: **Pololu A-Star 32U4** → Upload
4. Place robot at start, keep stationary ~1s for gyro calibration, then it runs autonomously

## Key Tuning Parameters

| Parameter | Description |
|---|---|
| `swerve_kP`, `swerve_kD`, `swerve_kA` | PID gains for turn control |
| `velocity_setpoint` | Base target speed (cm/s) |
| `BOT_RADIUS` | Half wheel-to-wheel distance (cm) |
| `ALPHA` | IMU filter coefficient (0–1) |
| `str_min` | Minimum straight-line PWM |
| `turnTime` | Duration for timed turn segments (s) |
| `delta_T` | Global target run time (s) |

## Results

🥈 2nd Place — MIT Science Olympiad Invitational

---

*[Jay Sahni](https://github.com/jaysahni) · Science Olympiad Division C*
