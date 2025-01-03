# Written by: Christopher Gholmieh & Saahir Kadri
# Imports:

# Phidget:
from Phidget22.Devices.Accelerometer import (Accelerometer)
from Phidget22.Devices.Gyroscope import (Gyroscope)

# Typing:
from typing import (Callable, Dict, List)


# IMU:
class IMU:
    # Initialization:
    def __init__(self, serial_number: int) -> None:
        # Acceleration:
        self.acceleration: List[float] = None

        # Timestamp:
        self.timestamp: float = 0

        # Rotation:
        self.rotation: List[float] = None
        
        # Gadgets:
        self.gadgets: Dict[str, Callable] = {
            # Accelerometer:
            "accelerometer": Accelerometer(),

            # Gyroscope:
            "gyroscope": Gyroscope()
        }

        # Serial:
        self.serial_number: int = serial_number

    # Methods:
    def initialize_handlers(self) -> None:
        # Acceleration:
        def on_acceleration_change_handler(_, acceleration: List[float], __) -> None:
            # Logic:
            self.acceleration = acceleration

        # Rotation:
        def on_angular_rate_update_handler(_, angular_rotation: List[float], timestamp: float) -> None:
            # Logic:
            self.timestamp = timestamp
            self.rotation = angular_rotation

        # Accelerometer:
        self.gadgets["accelerometer"].setOnAccelerationChangeHandler(on_acceleration_change_handler)
        
        # Gyroscope:
        self.gadgets["gyroscope"].setOnAngularRateUpdateHandler(on_angular_rate_update_handler)

    def initialize_gadgets(self) -> None:
        # Logic:
        for gadget in self.gadgets.values():
            # Serial:
            gadget.setDeviceSerialNumber(self.serial_number)

            # Initialization:
            gadget.openWaitForAttachment(1000)

            # Interval:
            gadget.setDataInterval(100)

    def get_acceleration(self) -> List[float]:
        return self.acceleration

    def get_timestamp(self) -> float:
        return self.timestamp
    
    def get_rotation(self) -> List[float]:
        return self.rotation

    def terminate(self) -> None:
        # Logic:
        for gadget in self.gadgets.values():
            # Close:
            gadget.close()