

# 🚗 Line Following Robot

## 📖 Overview

This project implements an autonomous **line-following robot** that uses a combination of **infrared sensors, PID control, and differential drive kinematics** to navigate along a predefined track. The system demonstrates the power of **feedback control** in robotics—continuously detecting errors and correcting its path in real time.

A line follower is one of the most fundamental yet elegant applications of robotics and control theory, bridging **mechanical design, electronics, and software** into a unified intelligent system.

---

## 🏗️ System Architecture

The robot consists of four primary subsystems:

### 1. Sensor Subsystem – *Eyes of the Robot*

* **5 IR sensors** arranged in a linear array (2 cm spacing)
* **Detection range:** 2–10 cm from ground
* **Response time:** <1 ms
* **Principle:** Detects line position by measuring reflectance difference (black absorbs IR, white reflects).

```
[S1] -- [S2] -- [S3] -- [S4] -- [S5]
 ↓       ↓       ↓       ↓       ↓
 0       0       1       0       0
```

---

### 2. Control Subsystem – *Brain of the Robot*

Implements a **PID controller** to minimize deviation from the line center.

**Control Law:**

```
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt
```

Where:

* `e(t)` = error (line position – setpoint)
* `u(t)` = steering correction
* `Kp, Ki, Kd` = controller gains

---

### 3. Actuation Subsystem – *Muscles of the Robot*

* **Differential drive:** left & right motors adjust speeds independently
* **Motor control logic:**

```
Left_Speed  = Base_Speed - Correction
Right_Speed = Base_Speed + Correction
```

---

### 4. Power & Communication Subsystem

* **Controller:** Arduino Uno (16 MHz)
* **Power:** 7.4 V Li-Po battery
* **Communication:** Serial debugging/monitoring

---

## 📐 Mathematical Model

### Error Calculation

```
Position = Σ(i × Sensor_Value[i]) / Σ(Sensor_Value[i])
Error    = Setpoint – Position
```

### Plant Model (Motor + Chassis Dynamics)

```
G(s) = K / (τs + 1)
```

### PID Transfer Function

```
C(s) = Kp + Ki/s + Kd·s
T(s) = (C(s)·G(s)) / (1 + C(s)·G(s))
```

---

## 🎯 Control Design Requirements

| Metric             | Target Value |
| ------------------ | ------------ |
| Rise Time (tr)     | ≤ 0.5 s      |
| Settling Time (ts) | ≤ 1.0 s      |
| Overshoot (Mp)     | ≤ 10%        |
| Steady-State Error | ≤ 2%         |
| Phase Margin       | ≥ 45°        |

Tuning strategy:

* **Kp** → immediate response (0.5–2.0)
* **Ki** → removes steady-state error (0.01–0.1)
* **Kd** → damping for overshoot (0.1–1.0)

---

## ⚙️ Hardware Specifications

| Component  | Specification                | Purpose        |
| ---------- | ---------------------------- | -------------- |
| Chassis    | 20 cm × 15 cm aluminum frame | Support        |
| Wheels     | 6.5 cm rubber                | Traction       |
| Motors     | 6 V DC, 100 RPM gear motors  | Drive          |
| Sensors    | QTR-5RC IR sensor array      | Line detection |
| Controller | Arduino Uno R3               | Processing     |
| Battery    | 7.4 V, 2200 mAh Li-Po        | Power          |
| Weight     | \~800 g                      | Stability      |

---

## 📊 Performance Targets

| Parameter            | Target Value |
| -------------------- | ------------ |
| Max Speed            | 0.5 m/s      |
| Min Turn Radius      | 15 cm        |
| Line Following Error | ±2 mm        |
| Response Time        | <100 ms      |
| Battery Life         | >2 hrs       |

---

## 🧩 Challenges & Solutions

| Challenge            | Solution                  |
| -------------------- | ------------------------- |
| Sensor noise         | Moving average filter     |
| Mechanical vibration | PID damping + Kd tuning   |
| Sharp turns          | Adaptive speed modulation |
| Speed vs accuracy    | Variable speed control    |

---

## 🚀 Innovations

* ✅ **Adaptive PID** – adjusts parameters dynamically
* ✅ **Look-ahead prediction** – smoother handling of turns
* ✅ **Fault detection** – monitors sensor reliability
* ✅ **Future-ready**: computer vision upgrade, wireless comms, swarm coordination

---

## 📂 Project Structure

```
line-follower-robot/
│── docs/                # Diagrams, block models
│── hardware/            # CAD, wiring diagrams
│── firmware/            # Arduino code
│── control/             # MATLAB/Simulink models
│── simulation/          # Robot simulation + animations
│── README.md            # Project overview
```

---

## 📸 Block Diagram

```
IR Sensors → Error Calc → PID Controller → Motor Driver → Motors
                           ↑
                           | Feedback (line position)
```

---

## 🧪 Validation Tools

* Step Response Analysis
* Nyquist & Bode Plots for stability margins
* Simulink model comparison: **open-loop vs. controlled response**
* MATLAB animation of robot following path

---



