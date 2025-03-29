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
    def __init__(self) -> None:
        # IMUs:
        self.thigh_imu = Spatial()
        self.shank_imu = Spatial()

        # Serial:
        self.thigh_imu.setDeviceSerialNumber(721783)
        self.shank_imu.setDeviceSerialNumber(721888)

        # Quaternions:
        self.thigh_quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self.shank_quaternion = np.array([1.0, 0.0, 0.0, 0.0])

    # Methods:
    def clamp_value(self, minimum: float, value: float, maximum: float) -> float:
        if value < minimum:
            return minimum
        
        if value > maximum:
            return maximum

        return value

    def calculate_flexion_angle(self, debug: bool = True) -> float:
        try:
            # Variables (Assignment):
            # Rotations:
            rotation_thigh = R.from_quat(self.thigh_quaternion)
            rotation_shank = R.from_quat(self.shank_quaternion)

            # Rotation:
            relative_rotation = rotation_shank * rotation_thigh.inv()

            # Euler:
            euler_angles = relative_rotation.as_euler("xyz", degrees=True)

            # Flexion:
            flexion_angle = euler_angles[2] # Z

            # Logic:
            if debug:
                logger.info(f"[Unclamped Flexion]: {flexion_angle}")

            flexion_angle = self.clamp_value(0.0, flexion_angle, 180.0)

            if debug:
                print(f"[Clamped Flexion]: {flexion_angle}")
                print(f"[PWM]: {map_angle_to_pwm(flexion_angle)}")

            return flexion_angle
        except Exception as exception:
            print(f"[Error | Calculation]: {exception}")

    def handle_thigh_imu(self, spatial, acceleration, angular_rotation, magnetic_field, timestamp) -> None:
        try:
            # Variables (Assignment):
            # Quaternion:
            quaternion = spatial.getQuaternion()

            # Logic:
            self.thigh_quaternion = np.array([quaternion.w, quaternion.x, quaternion.y, quaternion.z])
        except PhidgetException as exception:
            print(f"[Error | Thigh IMU Quaternion]: {exception}")

    def handle_shank_imu(self, spatial, acceleration, angular_rotation, magnetic_field, timestamp) -> None:
        try:
            # Variables (Assignment):
            # Quaternion:
            quaternion = spatial.getQuaternion()

            # Logic:
            self.shank_quaternion = np.array([quaternion.w, quaternion.x, quaternion.y, quaternion.z])

            self.calculate_flexion_angle(debug=True)
        except PhidgetException as exception:
            print(f"[Error | Shank IMU Quaternion]: {exception}")

    def start(self) -> None:
        try:
            # Initialization:
            self.thigh_imu.openWaitForAttachment(5000)
            self.shank_imu.openWaitForAttachment(5000)

            print("[+] IMUs connected. Starting data collection...")

            self.thigh_imu.setOnSpatialDataHandler(self.handle_thigh_imu)
            self.shank_imu.setOnSpatialDataHandler(self.handle_shank_imu)

            # Loop:
            while True:
                time.sleep(0.1)

        except PhidgetException as exception:
            print(f"[Phidget Exception]: {exception}")

        finally:
            self.thigh_imu.close()
            self.shank_imu.close()

            print("[*] IMUs disconnected.")
