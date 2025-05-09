# Written by: Christopher Gholmieh
# Imports:

# Phidget:
# Spatial:
from Phidget22.Devices.Spatial import *

# Phidget:
from Phidget22.Phidget import *

# Loguru:
from loguru import logger

# Numpy:
import numpy as np

# Time:
import time

# Math:
from math import (degrees, acos)

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
        
        # Orientations:
        self.thigh_orientation = np.array([0.0, 0.0, 1.0])
        self.shin_orientation = np.array([0.0, 0.0, 1.0])

        # Handlers:
        self.thigh_imu.setOnSpatialDataHandler(self.handle_thigh_imu_data)
        self.shank_imu.setOnSpatialDataHandler(self.handle_shank_imu_data)
        
        # Calibration:
        self.calibration_offset = 0.0

    # Methods:
    def handle_thigh_imu_data(self, spatial, acceleration, angularRate, magneticField, timestamp: float):
        # Array:
        self.thigh_orientation = np.array([acceleration[0], acceleration[1], acceleration[2]])

        # Normalize:
        self.thigh_orientation = self.thigh_orientation / np.linalg.norm(self.thigh_orientation)

    def handle_shank_imu_data(self, spatial, acceleration, angularRate, magneticField, timestamp):
        # Array:
        self.shin_orientation = np.array([acceleration[0], acceleration[1], acceleration[2]])

        # Normalize:
        self.shin_orientation = self.shin_orientation / np.linalg.norm(self.shin_orientation)

        # Logic:
        self.calculate_knee_angle()

    # Calibration:
    def calibrate(self):
        # NOTE: When calibrating, keep the knee fully extended for two seconds.

        # Interval:
        time.sleep(2)

        # Variables (Assignment):
        # Dot:
        dot_product = np.dot(self.thigh_orientation, self.shin_orientation)

        # Initial:
        initial_angle = degrees(acos(min(1.0, max(-1.0, dot_product))))

        # Offset:
        self.calibration_offset = initial_angle

        # Logging:
        logger.info("[+] Calibration completed.")

    def calculate_knee_angle(self):
        # Variables (Assignment):
        # Dot:
        dot_product = np.dot(self.thigh_orientation, self.shin_orientation)

        # Range:
        dot_product = min(1.0, max(-1.0, dot_product))

        # Offset:
        angle = degrees(acos(dot_product)) + self.calibration_offset

        # Range:
        angle = min(180.0, max(0.0, angle))

        # Convert angle to PWM

        pwm_value = map_angle_to_pwm(angle)
        
        # Logging:
        logger.info(f"[*] Knee flexion angle: {angle:.1f} degrees, PWM: {pwm_value}")

        # Logic:
        return angle, pwm_value

    # Start:
    def start(self):
        try:
            # Initialization:
            self.thigh_imu.openWaitForAttachment(5000)
            self.shank_imu.openWaitForAttachment(5000)
            
            # Logging:
            logger.info("[+] IMUs connected. Calibrating..")

            # Calibration:
            self.calibrate()

            # Logging:
            logger.info("[*] Starting calculation process..")
            
            # Input:
            input("Press Enter to stop...")
            
        except PhidgetException as exception:
            print(f"Phidget Exception {exception.code}: {exception.details}")
            
        finally:
            self.thigh_imu.close()
            self.shank_imu.close()
