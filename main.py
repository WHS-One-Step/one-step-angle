# Written by: Christopher Gholmieh & Saahir Kadri
# Imports:

# Core:
from core import (Tracker)


# Main:
if __name__ == "__main__":
    # Variables (Assignment):
    # Tracker:
    tracker: Tracker = Tracker()

    # Logic:
    tracker.start()

    angle, pwm = tracker.calculate_knee_angle()
    print(f"Knee Angle: {angle} -> PWM: {pwm}")