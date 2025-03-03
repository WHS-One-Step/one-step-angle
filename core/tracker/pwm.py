# Written by Saahir Kadri
# PWM Mapping Module 


# Maps knee flexion angle to a PWM signal value
def map_angle_to_pwm(angle: float) -> int:
    # Ensure angle is within bounds
    angle = max(0, min(180, angle))

    # Apply linear map formula

    pwm_value = 31 + ((angle * (255 - 31)) / 180)

    return int(pwm_value)
