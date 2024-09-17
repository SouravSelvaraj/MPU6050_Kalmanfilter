# MPU6050_Kalmanfilter
# Kalman Filter for MPU6050 on Jetson Orin

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
4. Install necessary Python packages:
   ```bash
   pip install smbus2 numpy
