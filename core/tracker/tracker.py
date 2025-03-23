# Written by: Christopher Gholmieh & Saahir Kadri
# Imports:

# Phidget:
from Phidget22.Devices.Spatial import *
from Phidget22.Phidget import *
from Phidget22.PhidgetException import PhidgetException

# Loguru:
from loguru import logger

# Numpy:
import numpy as np

# Scipy:
from scipy.spatial.transform import Rotation as R

# Time:
import time

# PWM:
from .pwm import map_angle_to_pwm


# Tracker:
class Tracker:
    # Initialization:
    def __init__(self):
        # IMUs:
        self.thigh_imu = Spatial()
        self.shank_imu = Spatial()

        # Serials:
        self.thigh_imu.setDeviceSerialNumber(721783)
        self.shank_imu.setDeviceSerialNumber(721888)

        # Quaternions:
        self.thigh_quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion
        self.shank_quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion

        # Handlers:
        self.thigh_imu.setOnSpatialDataHandler(self.handle_thigh_imu_data)
        self.shank_imu.setOnSpatialDataHandler(self.handle_shank_imu_data)

        # Calibration:
        self.calibration_offset = 0.0

    # Methods:
    def handle_thigh_imu_data(self, spatial, acceleration, angularRate, magneticField, timestamp: float):
        try:
            quaternion = spatial.getQuaternion()
            self.thigh_quaternion = np.array([quaternion[3], quaternion[0], quaternion[1], quaternion[2]])  # Convert to (w, x, y, z)
        except PhidgetException as e:
            logger.error(f"[!] Error reading thigh IMU quaternion: {e}")

    def handle_shank_imu_data(self, spatial, acceleration, angularRate, magneticField, timestamp):
        try:
            quaternion = spatial.getQuaternion()
            self.shank_quaternion = np.array([quaternion[3], quaternion[0], quaternion[1], quaternion[2]])  # Convert to (w, x, y, z)
            self.calculate_knee_angle()  # Calculate on new data
        except PhidgetException as e:
            logger.error(f"[!] Error reading shank IMU quaternion: {e}")

    # Calibration:
    def calibrate(self):
        logger.info("[*] Calibrating... Please keep the knee fully extended.")

        time.sleep(2)
        angles = []

        for _ in range(20):  # Average over 20 readings
            angle = self.calculate_knee_angle(raw=True)
            if angle is not None:
                angles.append(angle)
            time.sleep(0.1)

        if angles:
            self.calibration_offset = np.mean(angles)
            logger.info(f"[+] Calibration completed. Offset: {self.calibration_offset:.2f} degrees")
        else:
            logger.error("[!] Calibration failed. No valid readings.")

    def calculate_knee_angle(self, raw=False):
        try:
            # Compute relative rotation: R_shank * R_thigh^(-1)
            relative_rotation = R.from_quat(self.shank_quaternion) * R.from_quat(self.thigh_quaternion).inv()
            euler_angles = relative_rotation.as_euler('xyz', degrees=True)  # Convert to Euler angles

            knee_angle = euler_angles[1]  # Pitch (Y-axis rotation)
            knee_angle = max(0.0, min(180.0, knee_angle + self.calibration_offset))  # Apply offset and limit range

            if raw:
                return knee_angle  # Return raw value for calibration

            pwm_value = map_angle_to_pwm(knee_angle)  # Convert to PWM

            logger.info(f"[*] Knee flexion angle: {knee_angle:.1f} degrees, PWM: {pwm_value}")
            return knee_angle, pwm_value
        except Exception as e:
            logger.error(f"[!] Error calculating knee angle: {e}")
            return None

    # Start:
    def start(self):
        try:
            self.thigh_imu.openWaitForAttachment(5000)
            self.shank_imu.openWaitForAttachment(5000)

            logger.info("[+] IMUs connected. Starting calibration...")

            self.calibrate()

            logger.info("[*] Starting knee angle calculation... Press Enter to stop.")

            input()
        except PhidgetException as exception:
            logger.error(f"Phidget Exception {exception.code}: {exception.details}")
        finally:
            self.thigh_imu.close()
            self.shank_imu.close()
            logger.info("[*] IMUs disconnected.")
