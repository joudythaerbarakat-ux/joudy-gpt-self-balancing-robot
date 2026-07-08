# Joudy GPT — Self-Balancing Robot with BLE Control Interface

> **Presented at NICE 2026** — 11th National Innovation Competition in Engineering  
> Tishk International University, Erbil, Iraq | Dean's Certificate of Appreciation awarded

![Demo](docs/demo.gif)

---

## Overview

A two-wheeled self-balancing robot implementing a **Kalman-filtered PID controller** on ESP32, with a custom **Bluetooth web app** for real-time control and a **7-state animated LCD emotion display**.

Built and presented at the National Innovation Competition in Engineering (NICE 2026), Mechatronics Department (MECHAT 4), Tishk International University.

---

## Key Technical Features

- **Kalman Filter + PID** — sensor fusion of MPU6050 accelerometer and gyroscope for stable angle estimation; balance loop running at ~100 Hz
- **Dual operating modes** — RC mode (manual BLE control) and BALANCE mode (autonomous stabilisation with BLE directional trim)
- **Live PID tuning over BLE** — Kp, Ki, Kd, setpoint, and axis configuration adjustable at runtime without re-flashing firmware
- **7-state animated LCD face system** — IDLE (auto-blink), HAPPY, SAD, ANGRY, SURPRISED, TALKING (mouth animation), THINKING (asymmetric eye animation); all BLE-controllable in real time
- **Custom BLE web app** — HTML/CSS/JavaScript using Web Bluetooth API; D-pad control, 3 speed modes, 5 pre-programmed voice lines via browser Web Speech API

---

## Hardware

| Component | Role |
|---|---|
| ESP32 WROOM-32 | Main MCU, BLE host |
| MPU6050 IMU | 6-axis accelerometer + gyroscope |
| L298N Motor Driver | Dual DC motor PWM control |
| DC Geared Motors ×2 | Drive wheels |
| XL4015 Buck Converter | 14.8V → 5V power regulation |
| 4× 18650 Li-ion (14.8V, 4S) | Main power supply |
| LCD 16×2 I²C (0x27) | Animated emotion face display |
| PETG 3D-printed chassis | Custom SolidWorks design |

---

## Wiring Diagram

![Wiring Diagram](docs/wiring-diagram.png)

> ⚠️ Diagram is illustrative. Exact GPIO assignments are defined in `AI_ROBOT.ino` (lines 14–19).  
> Motor pins: IN1=14, IN2=27, ENA=12, IN3=13, IN4=15, ENB=33. I²C: SDA=21, SCL=22.

---

## Software Architecture

```
loop() ~100 Hz
├── handleSerialCommands()     # BLE + Serial command parser
├── balanceUpdate(dt)          # Core balance loop
│   ├── mpuRead()              # Raw IMU register read (I²C)
│   ├── getRawAccelAngle()     # atan2 angle from accel axes
│   ├── getRawGyroRate()       # Gyro rate with axis selection
│   ├── kalman.update()        # Discrete Kalman filter
│   └── motorsDriveRaw()       # L298N PWM output with deadband
└── updateFace()               # LCD state machine @ 10 Hz
```

---

## Control System

Discrete Kalman filter for angle estimation, fusing accelerometer (absolute angle) and gyroscope (rate integration):

| Parameter | Value |
|---|---|
| Process noise Q_angle | 0.001 |
| Process noise Q_bias | 0.003 |
| Measurement noise R_meas | 0.03 |
| Default Kp | 12.0 |
| Default Ki | 0.0 |
| Default Kd | 0.8 |
| Safety tilt cutout | ±35° |
| Integral windup limit | ±80 |
| Target loop rate | ~100 Hz |

---

## BLE Command Reference

| Command | Action |
|---|---|
| `F` / `B` / `L` / `R` / `S` | Move / Stop |
| `MODE:RC` | Manual remote control mode |
| `MODE:BALANCE` | Autonomous self-balance mode |
| `CAL` | Calibrate upright angle (300-sample average) |
| `KP:12.0` / `KI:0.0` / `KD:0.8` | Live PID tuning |
| `SP:0.0` | Adjust balance setpoint angle |
| `AXIS:X` / `AXIS:Y` | Select accelerometer tilt axis |
| `GYRO:X/Y/Z` | Select gyroscope rate axis |
| `INV:0` / `INV:1` | Invert angle sign |
| `MINPWM:80` | Motor deadband compensation |
| `DIAG:1` / `DIAG:0` | Serial diagnostic output |
| `FACE:HAPPY` etc. | Set LCD emotion |
| `SPD:1/2/3` | Set RC speed level |

---

## Repository Structure

```
joudy-gpt-self-balancing-robot/
├── AI_ROBOT.ino          # Main ESP32 firmware
├── web-app/
│   └── index.html        # BLE controller (open in Chrome on Android)
├── docs/
│   ├── demo.gif          # Live demonstration
│   └── wiring-diagram.png
├── cad/                  # SolidWorks chassis files (coming soon)
└── README.md
```

---

## Build & Flash

**Requirements:**
- Arduino IDE 2.x with ESP32 Arduino Core 3.x
- Libraries: `LiquidCrystal_I2C`, `BLEDevice` (ESP32 built-in)

**Steps:**
1. Open `AI_ROBOT.ino` in Arduino IDE
2. Select board: `ESP32 Dev Module`
3. Flash at 115200 baud
4. Open Serial Monitor → send `HELP` for full command reference
5. Open `web-app/index.html` in **Chrome on Android** → tap Connect

---

## Calibration Procedure

1. Place robot on a flat surface, hold **upright and still**
2. Send `CAL` via BLE or Serial Monitor
3. Robot averages 300 samples over ~1.5 seconds
4. Send `MODE:BALANCE`
5. Trim `SP:` value (e.g. `SP:1.5`) to correct forward/backward lean

---

## Competition

Presented at **NICE 2026** (11th National Innovation Competition in Engineering),  
Mechatronics Department (MECHAT 4), Tishk International University, Erbil, Iraq.  
Theme: *Targeting Smart Sustainable Innovations.*  
**Certificate of Appreciation** awarded by the Dean of Engineering Faculty, TIU.

---

## Author

   **Joudy Thaer Barakat**
   Mechatronics Engineering Student, Tishk International University
   [LinkedIn](https://linkedin.com/in/joudy-barakat) · joudythaerbarakat@gmail.com 
