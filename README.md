# 🛰️ Smart Cable Tracker: IoT Cable Drum Monitoring System

![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=cplusplus&logoColor=white)
![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![AVR](https://img.shields.io/badge/AVR-ATmega328P-red?style=for-the-badge)
![IoT](https://img.shields.io/badge/IoT-Telemetry%20%26%20Tracking-008080?style=for-the-badge)

An embedded IoT telemetry platform designed to track real-time cable deployment, thermal conditions, and geographic positioning on industrial cable drums.

Developed as an industrial case study for **ElSewedy Electric**.

---

## 📌 Features

- **Rotational Kinematics & Cable Distance Measurement**: Computes exact unwound or retracted cable length based on drum rotation, utilizing a 7.0 cm drum radius.
- **Sensor Fusion (Complementary + Kalman Filtering)**: Fuses high-rate MPU6050 6-DOF IMU data (gyroscope Z-axis integration + accelerometer tilt angle) with a discrete Kalman Filter to eliminate gyro drift and vibration noise.
- **Bare-Metal Register-Level Peripherals**: Features custom low-level register drivers for I2C communication (100 kHz TWI), ADC multi-channel sampling, and 4-bit 16x2 LCD display driving without third-party display libraries.
- **Real-Time GPS Location Tracking**: Parses NMEA `$GPRMC` sentences from an onboard GPS module via `SoftwareSerial` to extract precise latitude and longitude coordinates.
- **Ambient Thermal Safety Monitoring**: Measures ambient conditions via an LM35 precision temperature sensor to flag overheating risks on heavy cable reels.
- **Dual Display Telemetry**: Displays distance (cm) and temperature (°C) on a 16x2 character LCD while outputting full serial telemetry logs.

---

## 🧠 Signal Processing & Kinematics Architecture

### 1. Sensor Calibration & Fusion Pipeline
To prevent gyroscope drift over long operational cycles, the system executes a 100-sample calibration phase upon boot to calculate the zero-rate offset. Sensor fusion is performed in two sequential stages:

- **Complementary Filter (α = 0.98)**: Fuses high-frequency gyroscope integration with low-frequency accelerometer tilt:  
  `θ_fused = 0.98 × (θ_fused + ω_z × Δt) + 0.02 × θ_accel`

- **Discrete Kalman Filter**: Predicts state and error covariance, computes the dynamic Kalman gain (K), and refines the estimated rotation angle against measurement noise (R = 0.1, Q = 0.01).

### 2. Rotational Cable Length Calculation
Using the filtered rotation angle (θ), the unwound cable distance (Δd) is derived from the circular arc length formula:

`Δd = (θ × 2π × r_drum) / 360°`

*Where r_drum = 7.0 cm represents the drum radius.*
---

## 📐 System Architecture & Workflow

```text

┌────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                           HARDWARE SENSORS                                             │
├──────────────────────────────┬──────────────────────────────────────────┬──────────────────────────────┤
│ MPU6050 6-DOF IMU (I2C)      │ LM35 Temperature Sensor (ADC A2)         │ Ublox M6 GPS Module (UART)   │
└──────────────┬───────────────┴────────────────────┬─────────────────────┴──────────────┬───────────────┘
               │                                    │                                    │
               ▼                                    ▼                                    ▼
┌──────────────────────────────┐   ┌──────────────────────────────────┐   ┌──────────────────────────────┐
│ 100-Sample Gyro Calibration  │   │ ADC Sampling & Linear Scaling    │   │ SoftwareSerial Stream Read   │
└──────────────┬───────────────┘   └────────────────┬─────────────────┘   └──────────────┬───────────────┘
               │                                    │                                    │
               ▼                                    │                                    ▼
┌──────────────────────────────┐                    │                     ┌──────────────────────────────┐
│ Complementary + Kalman Filter│                    │                     │ NMEA Sentence Parsing        │
│ Angle Fusion (θ)             │                    │                     │ ($GPRMC Lat/Long Extractor)  │
└──────────────┬───────────────┘                    │                     └──────────────┬───────────────┘
               │                                    │                                    │
               ▼                                    │                                    │
┌──────────────────────────────┐                    │                                    │
│ Drum Kinematic Formula       │                    │                                    │
│ Δd = (θ × 2π × r) / 360°     │                    │                                    │
└──────────────┬───────────────┘                    │                                    │
               │                                    │                                    │
               └─────────────────────────┬──────────┴────────────────────────────────────┘
                                         │
                                         ▼
                        ┌─────────────────────────────────┐
                        │     DUAL TELEMETRY OUTPUT       │
                        ├─────────────────────────────────┤
                        │ • 16x2 LCD Display (4-bit Mode) │
                        │ • Serial Telemetry (9600 Baud)  │
                        └─────────────────────────────────┘
```

---

## 🛠️ Hardware Pinout & Wiring Configuration

| Component | Interface / Pins | Microcontroller Connection | Function |
|---|---|---|---|
| **MPU6050 IMU** | Hardware I2C (TWI) | `SDA` (A4) / `SCL` (A5) | 6-DOF rotational & tilt measurement |
| **GPS Module** | SoftwareSerial UART | `RX` $\to$ `A0` (PC0), `TX` $\to$ `A1` (PC1) | Position tracking (Lat / Long) |
| **LM35 Temp Sensor**| Analog Input | `A2` (PC2) | Thermal monitoring |
| **16x2 LCD Data** | 4-bit Parallel Bus | `PD4`–`PD7` (Pins 4–7) | Display telemetry data |
| **16x2 LCD Control**| Control Signals | `RS` $\to$ `PB0` (Pin 8), `EN` $\to$ `PB1` (Pin 9) | Screen register & enable control |

---

## ⚙️ Tech Stack & Dependencies

- **Programming Language:** C++ / Native C (AVR-GCC)
- **Microcontroller Core:** ATmega328P (Arduino Uno)
- **Hardware Protocols:** I2C / TWI (100 kHz), SoftwareSerial UART, Analog-to-Digital Conversion (ADC)
- **Signal Processing:** Kalman Filter, Complementary Filter, Linear Interpolation

---

## 🚀 How to Run the System

### 1. Hardware Assembly
1. Connect the **MPU6050** to the Arduino's hardware I2C pins (`A4`/`A5`).
2. Wire the **GPS Module** TX/RX to `A0` and `A1`.
3. Connect the **LM35 Sensor** output pin to Analog Pin `A2`.
4. Connect the **16x2 LCD** data pins `D4–D7` to Digital Pins `4–7` and control pins `RS`/`EN` to Pins `8`/`9`.

### 2. Software Flashing
1. Open the project sketch in the **Arduino IDE**.
2. Select **Arduino Uno** as the target board.
3. Select the appropriate COM port and click **Upload**.

### 3. Verification
1. Power the board and wait for the LCD to display `Initializing...` followed by `System Ready`.
2. Observe real-time distance and temperature updates on the LCD display.
3. Open the **Serial Monitor** at **9600 Baud** to inspect raw sensor data, calculated distances, and incoming GPS coordinates.

---

## 📂 Project Structure

```text
Smart_Cable_Tracker/
│
├── Smart_Cable_Tracker.ino   # Main embedded code (I2C, ADC, LCD, Kalman Filter & GPS)
└── README.md                 # Project documentation
```

---

## 📜 Certification & Context

Developed between **January 2025 and April 2025** as an IoT industrial engineering case study for **ElSewedy Electric**, focusing on cable reel logistics and automated deployment tracking.

---

## 👨‍💻 Author

**Mahmoud El-Adgham**  
Electrical & Electronic Engineering | AI & Machine Learning  

* **GitHub:** [El-adghamF90](https://github.com/El-adghamF90)
* **LinkedIn:** [Mahmoud EL-Adgham](https://www.linkedin.com/in/mahmoud-el-adgham/)
