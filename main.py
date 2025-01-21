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

def calculate(imu_one: IMU, imu_two: IMU, previous_timestamp: float) -> float:
    """
    Calculate the angle between two IMUs in 3D space.
    """

    if imu_one.get_acceleration() is None or imu_two.get_acceleration() is None:
        raise ValueError("Acceleration data unavailable from one or both IMUs.")
    if imu_one.get_rotation() is None or imu_two.get_rotation() is None:
        raise ValueError("Rotation data unavailable from one or both IMUs.")


    current_timestamp = imu_two.get_timestamp()
    delta_time = (current_timestamp - previous_timestamp) / 1000  


    accel_one = normalize_vector(array(imu_one.get_acceleration()))
    accel_two = normalize_vector(array(imu_two.get_acceleration()))

    dot_product = dot(accel_one, accel_two)
    dot_product = clip(dot_product, -1.0, 1.0) 
    accel_angle = arccos(dot_product)

    gyro_one = array(imu_one.get_rotation()) * delta_time
    gyro_two = array(imu_two.get_rotation()) * delta_time
    gyro_angle = (gyro_two - gyro_one)[2]  


    combined_angle = complementary_filter(
        acceleration_angle=accel_angle,
        gyroscope_angle=gyro_angle,
        delta=delta_time,
        alpha=0.96  
    )


    angle_degrees = (combined_angle * 180 / 3.141592653589793) % 360

    return angle_degrees

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
print("Hello, world!")