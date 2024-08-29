import time
from pathlib import Path
import numpy as np

base_mpu_path = Path('/sys/bus/i2c/devices/1-0069/iio:device1')

# Dicionário para os caminhos dos arquivos
sensor_files = {
    'accel': {
        'x': base_mpu_path / 'in_accel_x_raw',
        'y': base_mpu_path / 'in_accel_y_raw',
        'z': base_mpu_path / 'in_accel_z_raw'
    },
    'gyro': {
        'x': base_mpu_path / 'in_anglvel_x_raw',
        'y': base_mpu_path / 'in_anglvel_y_raw',
        'z': base_mpu_path / 'in_anglvel_z_raw'
    }
}

def read_sensor_file(file_path):
    try:
        with open(file_path, 'r') as file:
            return int(file.read().strip())
    except (FileNotFoundError, ValueError):
        return 0

def read_sensors(sensor_paths, scale):
    return [read_sensor_file(sensor_paths[axis]) / scale for axis in 'xyz']

def calibrate_sensors(sensor_paths, num_samples=100):
    accel_offsets = np.zeros(3)
    gyro_offsets = np.zeros(3)

    for _ in range(num_samples):
        accel_readings = read_sensors(sensor_paths['accel'], 2048.0)
        gyro_readings = read_sensors(sensor_paths['gyro'], 16.4)
        
        accel_offsets += np.array(accel_readings)
        gyro_offsets += np.array(gyro_readings)
        
        time.sleep(0.01)
    
    accel_offsets /= num_samples
    gyro_offsets /= num_samples
    
    return accel_offsets, gyro_offsets

def calculate_euler_angles(accel_readings, gyro_readings, dt, euler_angles):
    accel_angle_x = np.arctan2(accel_readings[1], accel_readings[2])
    accel_angle_y = np.arctan2(-accel_readings[0], np.sqrt(accel_readings[1]**2 + accel_readings[2]**2))
    
    euler_angles[0] += gyro_readings[0] * dt
    euler_angles[1] += gyro_readings[1] * dt
    euler_angles[2] += gyro_readings[2] * dt
    
    euler_angles[0] = 0.98 * euler_angles[0] + 0.02 * accel_angle_x
    euler_angles[1] = 0.98 * euler_angles[1] + 0.02 * accel_angle_y
    
    return euler_angles

# Calibração do sensor
print("Calibrando sensores, por favor mantenha o dispositivo parado...")
accel_offsets, gyro_offsets = calibrate_sensors(sensor_files)
print("Calibração concluída.")

euler_angles = [0.0, 0.0, 0.0]
prev_time = time.time()

while True:
    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time

    accel_read = read_sensors(sensor_files['accel'], 2048.0) - accel_offsets
    gyro_read = read_sensors(sensor_files['gyro'], 16.4) - gyro_offsets
    
    euler_angles = calculate_euler_angles(accel_read, gyro_read, dt, euler_angles)
    
    print(f"AX = {accel_read[0]:+8.3f}g AY = {accel_read[1]:+8.3f}g AZ = {accel_read[2]:+8.3f}g | "
          f"GX = {gyro_read[0]:+8.3f}o/s GY = {gyro_read[1]:+8.3f}o/s GZ = {gyro_read[2]:+8.3f}o/s | "
          f"Roll = {euler_angles[0]:+8.3f} Pitch = {euler_angles[1]:+8.3f} Yaw = {euler_angles[2]:+8.3f}")
    
    time.sleep(0.5)