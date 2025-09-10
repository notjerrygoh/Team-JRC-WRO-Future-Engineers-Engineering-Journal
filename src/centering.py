def align_center(left_target=56, right_target=52, tolerance=5):
    while True:
        left_dist = leftultrasonic.distance * 100
        right_dist = rightultrasonic.distance * 100
      
        if abs(left_dist - left_target) <= tolerance and abs(right_dist - right_target) <= tolerance:
            motorSpeed(0)
            pwm.set_servo_pulsewidth(servo_pin, 1500)  # straighten
            break

        if left_dist < left_target:
            pwm.set_servo_pulsewidth(servo_pin, 2000)
            motorSpeed(80)

        elif right_dist < right_target:
            pwm.set_servo_pulsewidth(servo_pin, 1000)
            motorSpeed(80)

        else:
            # If both distances are bigger, just go straight and let sensors stabilize
            pwm.set_servo_pulsewidth(servo_pin, 1500)
            motorSpeed(80)

        # Small delay to avoid overshoot
        time_sleep = 0.1
        sleep(time_sleep)
