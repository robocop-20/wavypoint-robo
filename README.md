# 🤖 Waypoint Navigation Robot with GPS & Compass

A sophisticated autonomous robot navigation system using **GPS (5Hz)**, **QMC5883P Compass**, and **automatic offset learning algorithm** for accurate waypoint-following on a PRIZM-based platform.

---

## 📋 Table of Contents

- [Features](#features)
- [Hardware](#hardware)
- [Flowchart](#flowchart)
- [Key Algorithms](#key-algorithms)
- [Getting Started](#getting-started)
- [Configuration](#configuration)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)

---

## ✨ Features

### 🎯 Autonomous Navigation
- Waypoint-based path following with GPS coordinates
- Real-time distance and bearing calculations using Haversine formula
- Automatic waypoint advancement when within 5 meters

### 🧭 Smart Compass Calibration
- **Automatic hard iron calibration** during startup (10-second spin)
- **Dynamic boresight offset learning** - compass auto-calibrates during movement
- Eliminates manual compass offset tuning

### 📡 GPS Integration
- **5Hz GPS update rate** for smooth navigation
- **EMA smoothing** (α=0.3) to reduce GPS noise
- Speed-based offset learning: uses GPS course as ground truth when moving > 3 km/h

### 🔄 Hybrid Heading Control
- **Proportional steering** (Kp = 2.0) when heading error < 25°
- **In-place rotation** when heading error > 25°
- Smooth transitions between modes

### 📊 Real-time Telemetry
- Bluetooth debug output showing heading, GPS, offset, error, and distance

---

## 🔧 Hardware

| Component | Model | Purpose |
|-----------|-------|---------|
| **Controller** | PRIZM  | Motor control |
| **Compass** | QMC5883P | Magnetic heading |
| **GPS Module** | UBLOX | Position & course |
| **Motors** | DC Motors | Differential drive |
| **Bluetooth** | HC-05 | Telemetry output |
| **Encoders** | Motor encoders | Odometry validation |

### Robot Specifications
- **Wheel Radius:** 45 mm
- **Wheel Base:** 340 mm
- **Magnetic Declination:** -0.6°
- **Baud Rate:** 9600 bps

---

## 📊 Flowchart

### View the Complete Navigation Flowchart:

📱 **[Interactive Flowchart - CLICK HERE](FLOWCHART.html)**

The flowchart shows the complete navigation logic including:
1. Setup Phase - Initialization and calibration
2. Main Loop - Bluetooth control and navigation
3. GPS Processing - Data smoothing and calculations
4. Auto-Offset Learning - Self-calibration algorithm
5. Navigation Control - Motor commands and steering

---

## 🧠 Key Algorithms

### 1. Auto-Offset Learning Algorithm ⭐

The innovative self-calibration system:

```
IF GPS Speed > 3 km/h:
    GPS_Course = Ground Truth (TRUE heading)
    Compass_Reading = Measured magnetic heading
    Offset_Error = GPS_Course - Compass_Reading
    
    IF First Movement:
        Snap offset immediately
    ELSE:
        Converge slowly (5% learning rate)
        dynamicBoresightOffset += offsetError * 0.05
```

**Why it works:**
- GPS course is accurate when moving at speed
- Compass learns true boresight offset gradually
- Filters out transient magnetic anomalies
- No manual calibration needed!

### 2. Axis Mapping (Fixes "West instead of South")

```cpp
const char SENSOR_AXIS_FORWARD = 'Y';  // Sensor Y → Robot Forward
const char SENSOR_AXIS_RIGHT   = 'X';  // Sensor X → Robot Right
```

### 3. GPS Smoothing

Exponential Moving Average with α=0.3 reduces noise while maintaining responsiveness.

### 4. Heading Control

```
IF |headingError| > 25°:
    Turn in place (pure rotation)
ELSE:
    Cruise with proportional steering (Kp = 2.0)
```

---

## 🚀 Getting Started

### Prerequisites
- Arduino IDE
- PRIZM libraries
- Adafruit QMC5883P library
- TinyGPSPlus library

### Installation

1. Clone the repository
2. Install required libraries in Arduino IDE
3. Edit waypoints in the code
4. Configure GPS baud rate and pins
5. Upload to your PRIZM controller

---

## ⚙️ Configuration

### GPS Setup
```cpp
GPS_Serial.begin(9600);  // GPS baud rate
// Pins: RX = 3, TX = 4 (SoftwareSerial)
```

### Compass Calibration
```cpp
const bool AUTO_LEARN_OFFSET = true;  // Enable auto-learning
const float declinationDeg = -0.6;    // Your location declination
```

### Navigation Parameters
```cpp
const int CRUISE_SPEED = 50;       // Motor speed (0-100)
const int TURN_SPEED = 45;         // Turn speed (0-100)
const float WHEEL_RADIUS = 0.045;  // meters
const float WHEEL_BASE = 0.34;     // meters
```

### Set Your Waypoints
```cpp
NavPoint waypoints = {
  {17.780223, 83.375458},  // Waypoint 1
  {17.780277, 83.375477},  // Waypoint 2
  {17.780322, 83.375368},  // Waypoint 3
  {17.780305, 83.375359}   // Waypoint 4
};
```

---

## 📱 Usage

### 1. Power On
- Robot performs 10-second compass calibration
- Prints "Ready. Auto-Align Active." via Bluetooth

### 2. Start Navigation
- Awaits GPS lock
- Automatically starts waypoint following

### 3. Monitor via Bluetooth
```
Hdg:045 GPS:042 Off:145 Err:-3 Dist:12.5
Hdg:046 GPS:041 Off:144 Err:-2 Dist:11.3
```

| Field | Meaning |
|-------|---------|
| **Hdg** | Current compass heading (0-359°) |
| **GPS** | GPS course |
| **Off** | Dynamic boresight offset |
| **Err** | Heading error to target |
| **Dist** | Distance to waypoint (meters) |

### 4. Stop Navigation
- Send 'S' or 's' via Bluetooth to stop

---

## 🐛 Troubleshooting

### Compass Heading Wrong Direction
**Solution:** Adjust axis mapping:
```cpp
const char SENSOR_AXIS_FORWARD = 'Y';  // Try 'X', '-X', '-Y'
const char SENSOR_AXIS_RIGHT   = 'X';  // Try 'Y', '-X', '-Y'
```

### GPS Not Updating
**Solution:** Check connection and baud rate
```cpp
GPS_Serial.begin(9600);  // Verify this matches your module
```

### Offset Not Converging
**Solution:** Ensure robot is moving > 3 km/h during initial movement.

### Robot Spinning Wrong Direction
**Solution:** Check motor invert settings
```cpp
prizm.setMotorInvert(RIGHT_MOTOR, 1);  // Toggle if needed
```

### Drifting Off Course
**Solution:** Calibrate wheel radius and base width with actual measurements.

---

## 🎯 Roadmap

- [ ] IMU integration (better odometry)
- [ ] Path planning optimization
- [ ] Magnetic anomaly detection
- [ ] Multi-waypoint upload via Bluetooth
- [ ] Web dashboard
- [ ] Obstacle avoidance

---

## 📧 Contact

For questions, open an Issue in this repository or check the Flowchart for navigation logic details.

---

**Built with ❤️ for autonomous robotics**
