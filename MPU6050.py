import numpy as np
import matplotlib.pyplot as plt
import smbus2
import time

# I2C address of the device
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B  # Power management register
ACCEL_XOUT_H = 0x3B  # Accelerometer data register  X-axis high byte
ACCEL_YOUT_H = 0x3D  # Accelerometer data register  Y-axis high byte
GYRO_XOUT_H = 0x43  # Gyroscope data register  X-axis high byte
GYRO_YOUT_H = 0x45  # Gyroscope data register  Y-axis high byte
GYRO_ZOUT_H = 0x47  # Gyroscope data register  Z-axis high byte

#setu up MMPU6050
def MPU6050_init():
    bus = smbus2.SMBus(1)  # Create an I2C bus object
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)  # Wake up the MPU6050
    return bus

# Read accelerometer and gyroscope data
def read_raw_data(bus, addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    value = (high << 8) + low
    # Convert to signed value
    if value > 32767:
        value -= 65536
    return value

# Initialize the MPU6050
bus = MPU6050_init()


# Read accelerometer and gyroscope data
def read_sensor_data(bus):
    accel_x = read_raw_data(bus, ACCEL_XOUT_H)
    accel_y = read_raw_data(bus, ACCEL_YOUT_H)
    accel_z = read_raw_data(bus, ACCEL_XOUT_H+4)

    gyro_x = read_raw_data(bus, GYRO_XOUT_H)
    gyro_y = read_raw_data(bus, GYRO_YOUT_H)
    gyro_z = read_raw_data(bus, GYRO_ZOUT_H)
    
    #convert the raw data to g's and degrees/second
    accel_x = accel_x / 16384.0
    accel_y = accel_y / 16384.0
    accel_z = accel_z / 16384.0
    
    gyro_x = gyro_x / 131.0
    gyro_y = gyro_y / 131.0
    gyro_z = gyro_z / 131.0

    return (accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)

# kalman filter variables of 2D
dt = 0.01  # time interval

# Initialize estimates (position, velocity for x and y)
x = np.array([[0], [0]])  # state vector for X (position and velocity)
y = np.array([[0], [0]])  # state vector for Y (position and velocity)
yaw = np.array([[0], [0]])  # state vector for Yaw (position and velocity)

P_x = np.array([[1, 0], [0, 1]])  # covariance matrix for X
P_y = np.array([[1, 0], [0, 1]])  # covariance matrix for Y
P_yaw = np.array([[1, 0], [0, 1]])  # covariance matrix for Yaw

A = np.array([[1, dt], [0, 1]])  # state transition matrix for X
H = np.array([[1, 0]])  # measurement matrix
Q = np.array([[0.001, 0], [0, 0.001]])  # process noise covariance matrix
R = np.array([[0.01]])  # measurement noise covariance matrix
I = np.array([[1, 0], [0, 1]])  # identity matrix

# Kalman filter
def kalman_filter(x, P, z):
    # Predict
    x = A @ x
    P = A @ P @ A.T + Q

    # Measurement update
    y = z - H @ x
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x = x + K @ y
    P = (I - K @ H) @ P

    return x, P

#main loop
while True:
    # Read sensor data
    accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = read_sensor_data(bus)
    
    # Kalman filter for X
    z_x = np.array([[accel_x]])
    x, P_x = kalman_filter(x, P_x, z_x)
    
    # Kalman filter for Y
    z_y = np.array([[accel_y]])
    y, P_y = kalman_filter(y, P_y, z_y)
    
    # Kalman filter for Yaw
    z_yaw = np.array([[gyro_z]])
    yaw, P_yaw = kalman_filter(yaw, P_yaw, z_yaw)
    # Position estimates are the first element of the state vectors]
    print(f'X: {x[0][0]:.2f}, Y: {y[0][0]:.2f}, Yaw: {yaw[0][0]:.2f}')
    
    time.sleep(dt)