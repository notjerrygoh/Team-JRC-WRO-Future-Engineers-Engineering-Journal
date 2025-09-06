# Run sudo pigpiod in terminal before running code
# PARKING TEST VERSION - Only executes parking functionality

from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
from time import time, sleep
import pigpio
import numpy as np
import cv2
from picamera2 import Picamera2

pi = pigpio.pi()

# Ultrasonic sensors
factory = PiGPIOFactory()
rightultrasonic = DistanceSensor(echo=27, trigger=22, max_distance=4, pin_factory=factory) #White (22) and Brown (27)
leftultrasonic = DistanceSensor(echo=17, trigger=4, max_distance=4, pin_factory=factory) #Orange (17) and Blue (4)
frontultrasonic = DistanceSensor(echo=5, trigger=6, max_distance=4, pin_factory=factory) #Green (6) and Blue (5)
backultrasonic = DistanceSensor(echo=19, trigger=26, max_distance=4, pin_factory=factory) #white (19) and purple (26)

# Servo
servo_pin = 16
pwm = pigpio.pi()
pwm.set_mode(servo_pin, pigpio.OUTPUT)
pwm.set_PWM_frequency(servo_pin, 50)
pwm.set_servo_pulsewidth(servo_pin, 1500)  # neutral start

# Camera
cam = Picamera2()
cam.configure(cam.create_preview_configuration({"size": (640, 360)}))
cam.start()

# Motors
M1A = 23
M1B = 24
pi.set_mode(M1A, pigpio.OUTPUT)
pi.set_mode(M1B, pigpio.OUTPUT)

# Enhanced parking detection - finds multiple pink blocks and selects closest by area
def detect_parking_blocks(frame):
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    pink_lower = np.array([140, 100, 100], np.uint8)
    pink_upper = np.array([170, 255, 255], np.uint8)
    pink_mask = cv2.inRange(hsvFrame, pink_lower, pink_upper)

    contours, _ = cv2.findContours(pink_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    blocks = []
    
    if contours:
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:  # Minimum area for a block
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 255), 2)
                center_x = x + w // 2
                center_y = y + h // 2
                distance_from_center = abs(center_x - frame.shape[1]//2)
                blocks.append({
                    'center_x': center_x,
                    'center_y': center_y,
                    'x': x,
                    'y': y,
                    'width': w,
                    'height': h,
                    'area': area,
                    'distance_from_center': distance_from_center
                })
                
                # Label each block with its area for debugging
                cv2.putText(frame, f"Area: {int(area)}", (x, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Sort blocks by area (largest first) - closest blocks appear larger in camera
    blocks.sort(key=lambda b: b['area'], reverse=True)
    
    return blocks

# Motor speed
def motorSpeed(speed):
    speed *= 3
    if speed < 0:
        pi.set_PWM_dutycycle(M1A, abs(speed))
        pi.set_PWM_dutycycle(M1B, 0)
    elif speed > 0:
        pi.set_PWM_dutycycle(M1A, 0)
        pi.set_PWM_dutycycle(M1B, abs(speed))
    else:
        pi.set_PWM_dutycycle(M1A, 0)
        pi.set_PWM_dutycycle(M1B, 0)

# Parking state machine with forward-then-reverse parking
class ParkingStateMachine:
    def __init__(self):
        self.state = "APPROACH_CLOSEST_BLOCK"  # APPROACH_CLOSEST_BLOCK, STOP_10CM, MOVE_FORWARD, ALIGN_PARALLEL, REVERSE_PARK, PARKED
        self.closest_block = None
        self.maneuver_start_time = None
        
    def execute_parking(self, frame, distance_cm, current_time):
        blocks = detect_parking_blocks(frame)
        
        print(f"Parking state: {self.state}, Distance: {distance_cm:.1f}cm, Blocks found: {len(blocks)}")
        
        if self.state == "APPROACH_CLOSEST_BLOCK":
            if blocks:
                # Select the closest block (largest area in camera view)
                self.closest_block = blocks[0]
                frame_center = frame.shape[1] // 2
                block_center = self.closest_block['center_x']
                
                print(f"Targeting closest block - Area: {self.closest_block['area']}, Center: {block_center}")
                
                # Navigate towards the closest block
                if abs(block_center - frame_center) > 30:  # Not centered on block
                    if block_center < frame_center:
                        pwm.set_servo_pulsewidth(servo_pin, 1200)  # Turn left gently
                    else:
                        pwm.set_servo_pulsewidth(servo_pin, 1800)  # Turn right gently
                    motorSpeed(60)
                else:
                    # Centered on block, move forward
                    pwm.set_servo_pulsewidth(servo_pin, 1500)
                    if distance_cm > 10:
                        motorSpeed(60)
                    else:
                        # Reached 10cm distance
                        motorSpeed(0)
                        self.state = "STOP_10CM"
                        self.maneuver_start_time = current_time
                        print("Stopped 10cm from block - diagonal view achieved")
            else:
                # No blocks detected, search by rotating
                print("Searching for pink blocks...")
                # motorSpeed(0)
                pwm.set_servo_pulsewidth(servo_pin, 2000)  # Turn to search for blocks
                sleep(0.1)  # Small delay to allow turning
                
        elif self.state == "STOP_10CM":
            # Brief pause at 10cm to stabilize
            motorSpeed(0)
            pwm.set_servo_pulsewidth(servo_pin, 1500)
            if current_time - self.maneuver_start_time > 0.5:  # 0.5 second pause
                self.state = "MOVE_FORWARD"
                self.maneuver_start_time = current_time
                print("Moving forward to position for parallel parking")
                
        elif self.state == "MOVE_FORWARD":
            # Move forward past the block to position for parallel parking
            elapsed = current_time - self.maneuver_start_time
            if elapsed < 1.5:  # Move forward for 1.5 seconds
                pwm.set_servo_pulsewidth(servo_pin, 1500)  # Straight
                motorSpeed(80)
            else:
                motorSpeed(0)
                self.state = "ALIGN_PARALLEL"
                self.maneuver_start_time = current_time
                print("Positioning complete, aligning parallel to blocks")
                
        elif self.state == "ALIGN_PARALLEL":
            # Turn to align parallel with the parking blocks
            elapsed = current_time - self.maneuver_start_time
            if elapsed < 1.0:  # Turn for 1 second to align parallel
                pwm.set_servo_pulsewidth(servo_pin, 1000)  # Turn left to align parallel
                motorSpeed(50)
            else:
                pwm.set_servo_pulsewidth(servo_pin, 1500)
                motorSpeed(0)
                self.state = "REVERSE_PARK"
                self.maneuver_start_time = current_time
                print("Starting reverse parking maneuver")
                
        elif self.state == "REVERSE_PARK":
            # Reverse into the parking space with steering
            left_dist = leftultrasonic.distance * 100
            right_dist = rightultrasonic.distance * 100
            back_dist = backultrasonic.distance * 100
            
            print(f"Reverse parking - L:{left_dist:.1f} R:{right_dist:.1f} B:{back_dist:.1f}")
            
            # Check if we have space behind us
            if back_dist > 8:  # Still room to reverse (8cm safety margin)
                # Steer while reversing to enter the parking space
                elapsed = current_time - self.maneuver_start_time
                
                if elapsed < 2.0:  # First 2 seconds: turn into parking space
                    pwm.set_servo_pulsewidth(servo_pin, 2000)  # Turn right while reversing
                    motorSpeed(-70)  # Reverse
                elif elapsed < 4.0:  # Next 2 seconds: straighten out
                    pwm.set_servo_pulsewidth(servo_pin, 1500)  # Straight
                    motorSpeed(-70)  # Continue reversing
                else:
                    # Use ultrasonic sensors for final positioning
                    if left_dist < 8:  # Too close to left wall
                        pwm.set_servo_pulsewidth(servo_pin, 1800)  # Steer away from left
                    elif right_dist < 8:  # Too close to right wall
                        pwm.set_servo_pulsewidth(servo_pin, 1200)  # Steer away from right
                    else:
                        pwm.set_servo_pulsewidth(servo_pin, 1500)  # Go straight
                    motorSpeed(-60)  # Slower reverse for precision
            else:
                # Close to back wall or obstacle, stop parking
                motorSpeed(0)
                pwm.set_servo_pulsewidth(servo_pin, 1500)
                self.state = "PARKED"
                print("Parking completed successfully!")
                
        elif self.state == "PARKED":
            # Final state - stay parked
            motorSpeed(0)
            pwm.set_servo_pulsewidth(servo_pin, 1500)
            print("Robot successfully parked!")

# Initialize parking system
parking_machine = ParkingStateMachine()

print("=== PARKING TEST MODE ===")
print("Robot will immediately start parking sequence")
print("Press 'q' to quit")
print("=" * 30)

try:
    while True:
        frame = cv2.cvtColor(cam.capture_array()[::-1, :, :3], cv2.COLOR_RGB2BGR)
        distance_cm = frontultrasonic.distance * 100
        current_time = time()

        # Execute parking sequence
        parking_machine.execute_parking(frame, distance_cm, current_time)
        
        # Show parking detection
        # cv2.imshow("Parking Test", frame)

        # Exit condition
        if cv2.waitKey(10) & 0xFF == ord('q'):
            motorSpeed(0)
            break
            
        # Auto-exit when parked for testing
        if parking_machine.state == "PARKED":
            print("Parking test completed. Press 'q' to exit or wait 5 seconds...")
            sleep(5)
            break

finally:
    print("Stopping robot and cleaning up...")
    motorSpeed(0)
    pwm.set_servo_pulsewidth(servo_pin, 1500)  # Center servo
    sleep(0.5)
    pwm.set_servo_pulsewidth(servo_pin, 0)     # Turn off servo
    cv2.destroyAllWindows()
    cam.stop()
    pi.stop()
    print("Test complete!")
    
