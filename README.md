# 🧭 Embedded DSP IMU Sensor Fusion (Extended Kalman Filter)

[![Language](https://img.shields.io/badge/Language-C99%20%2F%20DSP-00599C?style=flat-square&logo=c&logoColor=white)](https://en.wikipedia.org/wiki/C99)
[![Algorithm](https://img.shields.io/badge/Algorithm-Extended%20Kalman%20Filter%20(EKF)-E0234E?style=flat-square)](https://en.wikipedia.org/wiki/Extended_Kalman_filter)
[![Target](https://img.shields.io/badge/Target-ARM%20Cortex--M4%20(STM32%20%2F%20CMSIS--DSP)-03234C?style=flat-square&logo=stmicroelectronics&logoColor=white)](https://www.st.com/)
[![License](https://img.shields.io/badge/License-MIT-blue?style=flat-square)](LICENSE)

A high-speed, deterministic **6-DOF / 9-DOF IMU Sensor Fusion Engine** implemented in ANSI C. Designed for real-time attitude & heading reference systems (AHRS), drones, robotics, and gimbal stabilization on **ARM Cortex-M4 (STM32F4)** microcontrollers.

---

## 🏛️ System Architecture

```mermaid
graph LR
    subgraph Sensors ["Sensor Input (100 Hz)"]
        G["3-Axis Gyroscope (deg/s)"]
        A["3-Axis Accelerometer (g)"]
    end

    subgraph EKF ["Extended Kalman Filter Pipeline"]
        P["State Prediction (Quaternion Kinematics)"]
        O["Outlier Rejection (Norm Bounds)"]
        K["Innovation & Kalman Gain Update"]
        N["Fast InvSqrt Normalization"]
    end

    subgraph Output ["Attitude Output (Degrees)"]
        E["Euler Angles: Roll / Pitch / Yaw"]
        Q["Unit Quaternion [w, x, y, z]"]
    end

    G --> P
    P --> K
    A --> O
    O --> K
    K --> N
    N --> E
    N --> Q
```

---

## ⚡ Core Features

1. **Gimbal Lock-Free Quaternion Representation**:
   - Represents 3D orientation using unit quaternions ($q_0, q_1, q_2, q_3$), eliminating singularity lock.
   - Integrates Carmack's fast inverse square root (`Fast_InvSqrt`) for sub-microsecond vector normalization.
2. **Gyroscope Bias & Dynamic Drift Rejection**:
   - Continuously estimates and subtracts zero-rate gyro drift over time.
   - Dynamic acceleration gate rejects linear shocks and vibration spikes ($0.2g < ||a|| < 2.0g$).
3. **Optimized for Embedded MCUs**:
   - Zero dynamic memory allocation (`malloc`).
   - RAM footprint: **< 1.2 KB**.
   - Execution time per update step on STM32F401 (84 MHz): **< 28 µs**.

---

## 🛠️ Build & Verification

```bash
# Compile using GCC
gcc -O2 -Wall -Wextra -Iinclude src/quaternion_math.c src/ekf_fusion.c src/main.c -lm -o dsp_ekf_fusion

# Run simulation
./dsp_ekf_fusion

# Run Python visualizer
python tools/attitude_visualizer.py
```

---

## 👤 Author
* **Herambeswar Mandadapu** – [@mandadapuherambeswar-ux](https://github.com/mandadapuherambeswar-ux)
* **LinkedIn:** [Herambeswar Mandadapu](https://linkedin.com/in/herambeswar-mandadapu-5a977a385)
