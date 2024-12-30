# Written by: Christopher Gholmieh & Saahir Kadri
# Imports:

# Loguru:
from loguru import (logger)

# Numpy:
from numpy.linalg import (norm as normalize)
from numpy import (ndarray, arccos, array, clip, dot)

# Time:
from time import (sleep)

# IMU:
from core import (IMU)

# Functions:
def complementary_filter(acceleration_angle: float, gyroscope_angle: float, delta: float, alpha: float = 0.96):
    return alpha * (gyroscope_angle + delta) + (1 - alpha) * acceleration_angle

def normalize_vector(vector: ndarray) -> ndarray:
    return vector / normalize(vector)

def calculate(imu_one: IMU, imu_two: IMU, previous_timestamp: float) -> None:
    # Validation:
    # IMU:
    if imu_one.get_acceleration() is None:
        pass

    if imu_one.get_rotation() is None:
        pass

    # IMU: 
    if imu_two.get_acceleration() is None:
        pass

    if imu_two.get_rotation() is None:
        pass

    # Variables (Assignment):
    # Delta:
    delta: float = (imu_two.get_timestamp() - previous_timestamp) / 1000

    # Accelerations:
    normalized_acceleration_one: ndarray = normalize_vector(array(imu_one.get_acceleration()))
    normalized_acceleration_two: ndarray = normalize_vector(array(imu_two.get_acceleration()))

    # Cosine:
    cosine_angle: float = dot(normalized_acceleration_one, normalized_acceleration_two)
    cosine_angle: float = clip(cosine_angle, -1.0, 1.0)

    # Acceleration:
    acceleration_angle: float = arccos(cosine_angle)

    # Velocity:
    angular_velocity_difference: ndarray = array(imu_two.get_acceleration()) - array(imu_one.get_acceleration())

    # Gyroscope:
    gyroscopical_angle: float = angular_velocity_difference[2] * delta

    # Logic:
    return complementary_filter(acceleration_angle, gyroscopical_angle, delta)

def main() -> None:
    # Variables (Assignment):
    IMU_one_serial_number: int = int(input("Enter first serial number: "))
    IMU_two_serial_number: int = int(input("Enter the second serial number: "))

    IMU_one: IMU = IMU(serial_number=IMU_one_serial_number)
    IMU_two: IMU = IMU(serial_number=IMU_two_serial_number)

    # Timestamp:
    previous_timestamp: float = IMU_two.get_timestamp()

    # Initialize:
    IMU_one.initialize_gadgets()
    IMU_one.initialize_handlers()

    IMU_two.initialize_gadgets()
    IMU_two.initialize_handlers()

    # Logic:
    try:
        while True:
            # Logic:
            logger.info("Angle: " + str(calculate(IMU_one, IMU_two, previous_timestamp)))

            # Sleep:
            sleep(0.1)
    except (Exception, KeyboardInterrupt):
        # Termiantion:
        IMU_one.terminate()
        IMU_two.terminate()

# Main:
main()