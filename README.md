# Kalman Filter for MPU6050 

This project implements a Kalman filter to estimate 2D position (X, Y) and yaw (rotation around the Z-axis) using the MPU6050 sensor on a Jetson Orin in Python. The Kalman filter fuses accelerometer and gyroscope data to provide smooth estimates of position and yaw, filtering out noise and sensor drift.

## Features
- **2D Position Estimation**: Tracks X and Y positions using accelerometer data.
- **Yaw Estimation**: Tracks yaw (Z-axis rotation) using gyroscope data.
- **Kalman Filter**: Uses a state-space model to fuse noisy sensor measurements.
- **Real-Time Sensor Fusion**: Combines accelerometer and gyroscope data for accurate motion tracking.

## Prerequisites
1. **Jetson Orin** (or another platform with an I2C interface).
2. **MPU6050** sensor (accelerometer + gyroscope).
3. **Python 3.x** installed.
4. Install necessary Python libraries.

##Hardware Setup
Connect the MPU6050 to the Jetson Orin using the I2C interface:
 - VCC: 3.3V
 - GND: Ground
 - SCL: I2C clock (Pin 5 on Jetson Orin)
 - SDA: I2C data (Pin 3 on Jetson Orin)
Ensure that the MPU6050 I2C address is set to 0x68 (default).

## Getting Started

### 1. Clone the Repository
Start by cloning the repository to your local machine:
```bash
git clone https://github.com/SouravSelvaraj/MPU6050_Kalmanfilter.git
cd MPU6050_Kalmanfilter.git
```

### 2. Install Dependencies
Install the necessary Python packages by running:
```bash
pip install smbus2 numpy
```
### 3. Initialize the MPU6050
Make sure the MPU6050 is properly connected and powered. The sensor is configured via I2C using the smbus2 library. Ensure that the I2C address of the MPU6050 is set to the default 0x68.

### 4. Running the Kalman Filter
Run the script to start reading the MPU6050 sensor data and estimating the position (X, Y) and yaw (rotation around the Z-axis) in real-time:
```bash
python MPU6050.py
```
