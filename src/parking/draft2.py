# Run sudo pigpiod in terminal before running code
# IMPROVED PARKING VERSION - Follows 6-step parking sequence

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

# Detect pink parking blocks (magenta RGB 255, 0, 255)
def detect_parking_blocks(frame):
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Adjusted for magenta/pink detection
    pink_lower = np.array([140, 50, 50], np.uint8)
    pink_upper = np.array([170, 255, 255], np.uint8)
    pink_mask = cv2.inRange(hsvFrame, pink_lower, pink_upper)

    contours, _ = cv2.findContours(pink_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    blocks = []
    
    if contours:
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 800:  # Minimum area for a block
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 255), 2)
                center_x = x + w // 2
                center_y = y + h // 2
                blocks.append({
                    'center_x': center_x,
                    'center_y': center_y,
                    'x': x,
                    'y': y,
                    'width': w,
                    'height': h,
                    'area': area
                })
                
                cv2.putText(frame, f"Block: {int(area)}", (x, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Sort by area (largest first - closest block)
    blocks.sort(key=lambda b: b['area'], reverse=True)
    return blocks

# Motor control function
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

# Six-step parking state machine
class SixStepParkingMachine:
    def __init__(self):
        self.state = "STEP1_APPROACH"  # STEP1_APPROACH, STEP2_TURN_PARALLEL, STEP3_MOVE_AHEAD, 
                                       # STEP4_INITIAL_PARK, STEP5_FORWARD_REVERSE, STEP6_FINAL_POSITION
        self.step_start_time = None
        self.target_block = None
        self.movement_start_time = None
        
    def get_sensor_readings(self):
        """Get all ultrasonic sensor readings in cm"""
        return {
            'front': frontultrasonic.distance * 100,
            'back': backultrasonic.distance * 100,
            'left': leftultrasonic.distance * 100,
            'right': rightultrasonic.distance * 100
        }
        
    def execute_parking(self, frame, current_time):
        blocks = detect_parking_blocks(frame)
        sensors = self.get_sensor_readings()
        
        print(f"Step: {self.state}")
        print(f"Sensors - F:{sensors['front']:.1f} B:{sensors['back']:.1f} L:{sensors['left']:.1f} R:{sensors['right']:.1f}")
        
        if self.state == "STEP1_APPROACH":
            # Step 1: Move towards the blocks until 10cm away
            if blocks:
                self.target_block = blocks[0]  # Largest block (closest)
                frame_center = frame.shape[1] // 2
                block_center = self.target_block['center_x']
                
                # Minor steering to stay aimed at block
                if abs(block_center - frame_center) > 50:
                    if block_center < frame_center:
                        pwm.set_servo_pulsewidth(servo_pin, 1400)  # Slight left
                    else:
                        pwm.set_servo_pulsewidth(servo_pin, 1600)  # Slight right
                else:
                    pwm.set_servo_pulsewidth(servo_pin, 1500)  # Straight
                
                # Check distance to block using front ultrasonic
                if sensors['front'] > 10:
                    motorSpeed(60)  # Move forward
                    print(f"Approaching block, distance: {sensors['front']:.1f}cm")
                else:
                    # Reached 10cm - stop and prepare for step 2
                    motorSpeed(0)
                    pwm.set_servo_pulsewidth(servo_pin, 1500)
                    self.state = "STEP2_TURN_PARALLEL"
                    self.step_start_time = current_time
                    print("STEP 1 COMPLETE: 10cm from block, starting turn to parallel")
            else:
                # Search for blocks with gentle movements
                print("Searching for pink blocks...")
                motorSpeed(25)  # Very slow search speed
                pwm.set_servo_pulsewidth(servo_pin, 1600)  # Gentle turn to search
                
        elif self.state == "STEP2_TURN_PARALLEL":
            # Step 2: Turn right until parallel with parking lot
            if self.step_start_time is None:
                self.step_start_time = current_time
                
            elapsed = current_time - self.step_start_time
            
            if elapsed < 2.5:  # Turn for 2.5 seconds to become parallel
                pwm.set_servo_pulsewidth(servo_pin, 1900)  # Turn right
                motorSpeed(50)  # Move while turning
                print(f"Turning parallel to parking lot... {elapsed:.1f}s")
            else:
                # Finished turning
                motorSpeed(0)
                pwm.set_servo_pulsewidth(servo_pin, 1500)
                self.state = "STEP3_MOVE_AHEAD"
                self.step_start_time = current_time
                print("STEP 2 COMPLETE: Now parallel with parking lot")
                
        elif self.state == "STEP3_MOVE_AHEAD":
            # Step 3: Move straight ahead about 10cm to position for parking
            if self.step_start_time is None:
                self.step_start_time = current_time
                
            elapsed = current_time - self.step_start_time
            
            if elapsed < 1.2:  # Move forward for 1.2 seconds (approximately 10cm)
                pwm.set_servo_pulsewidth(servo_pin, 1500)  # Straight
                motorSpeed(50)
                print(f"Moving ahead of parking space... {elapsed:.1f}s")
            else:
                motorSpeed(0)
                self.state = "STEP4_INITIAL_PARK"
                self.step_start_time = current_time
                print("STEP 3 COMPLETE: Positioned ahead of parking space")
                
        elif self.state == "STEP4_INITIAL_PARK":
            # Step 4: Turn left and park until too close to walls (5cm)
            if self.step_start_time is None:
                self.step_start_time = current_time
                
            # Turn left and move into parking space
            pwm.set_servo_pulsewidth(servo_pin, 1100)  # Turn left
            
            # Check if too close to any wall (5cm safety)
            if (sensors['front'] > 5 and sensors['back'] > 5 and 
                sensors['left'] > 5 and sensors['right'] > 5):
                motorSpeed(-60)  # Reverse into space
                print("Parking into space...")
            else:
                motorSpeed(0)
                pwm.set_servo_pulsewidth(servo_pin, 1500)
                self.state = "STEP5_FORWARD_REVERSE"
                self.step_start_time = current_time
                print("STEP 4 COMPLETE: Initial parking done, too close to wall")
                
        elif self.state == "STEP5_FORWARD_REVERSE":
            # Step 5: Forward 2 seconds, turn right, reverse until too close
            if self.step_start_time is None:
                self.step_start_time = current_time
                self.movement_start_time = current_time
                
            elapsed = current_time - self.step_start_time
            
            if elapsed < 2.0:  # Forward for 2 seconds
                pwm.set_servo_pulsewidth(servo_pin, 1500)  # Straight
                motorSpeed(60)  # Same speed as before
                print(f"Moving forward for 2 seconds... {elapsed:.1f}s")
            elif elapsed < 2.5:  # Brief pause to turn servo
                motorSpeed(0)
                pwm.set_servo_pulsewidth(servo_pin, 1900)  # Turn right
                print("Turning servo right...")
            else:  # Now reverse until too close
                if (sensors['front'] > 5 and sensors['back'] > 5 and 
                    sensors['left'] > 5 and sensors['right'] > 5):
                    motorSpeed(-60)  # Reverse
                    print("Reversing with right turn...")
                else:
                    motorSpeed(0)
                    self.state = "STEP6_FINAL_POSITION"
                    self.step_start_time = current_time
                    print("STEP 5 COMPLETE: Forward-reverse maneuver done")
                    
        elif self.state == "STEP6_FINAL_POSITION":
            # Step 6: Turn left, move forward to final position
            # Target: 7cm between front/back walls, 2cm from one side
            if self.step_start_time is None:
                self.step_start_time = current_time
                
            pwm.set_servo_pulsewidth(servo_pin, 1100)  # Turn left
            
            front_back_diff = abs(sensors['front'] - sensors['back'])
            min_side_distance = min(sensors['left'], sensors['right'])
            
            print(f"Final positioning - Front/Back diff: {front_back_diff:.1f}, Min side: {min_side_distance:.1f}")
            
            # Check if in final position
            if front_back_diff > 7 or min_side_distance > 2.5:
                motorSpeed(40)  # Slow forward movement for precision
                print("Fine-tuning final position...")
            else:
                # Perfect position achieved
                motorSpeed(0)
                pwm.set_servo_pulsewidth(servo_pin, 1500)
                print("STEP 6 COMPLETE: PERFECT PARKING ACHIEVED!")
                print(f"Final sensors - F:{sensors['front']:.1f} B:{sensors['back']:.1f} L:{sensors['left']:.1f} R:{sensors['right']:.1f}")
                return True  # Parking complete
                
        return False  # Parking still in progress

# Initialize the six-step parking machine
parking_system = SixStepParkingMachine()

print("=== SIX-STEP PARKING SEQUENCE ===")
print("1. Approach block to 10cm")
print("2. Turn parallel to parking lot") 
print("3. Move ahead of parking space")
print("4. Initial park until close to walls")
print("5. Forward 2s, turn right, reverse")
print("6. Turn left, final positioning")
print("Press 'q' to quit")
print("=" * 35)

try:
    while True:
        frame = cv2.cvtColor(cam.capture_array()[::-1, :, :3], cv2.COLOR_RGB2BGR)
        current_time = time()

        # Execute the six-step parking sequence
        parking_complete = parking_system.execute_parking(frame, current_time)
        
        # Show camera feed with enhanced block detection
        cv2.imshow("Six-Step Parking - Pink Detection", frame)

        # Exit condition
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
            
        # Auto-exit when parking is perfectly complete
        if parking_complete:
            print("🎉 PARKING SEQUENCE SUCCESSFULLY COMPLETED! 🎉")
            print("Waiting 3 seconds before exit...")
            sleep(3)
            break
            
        sleep(0.1)  # Small delay for stability

except KeyboardInterrupt:
    print("Manual stop requested")

finally:
    print("Stopping robot and cleaning up...")
    motorSpeed(0)
    pwm.set_servo_pulsewidth(servo_pin, 1500)  # Center servo
    sleep(0.5)
    pwm.set_servo_pulsewidth(servo_pin, 0)     # Turn off servo
    cv2.destroyAllWindows()
    cam.stop()
    pi.stop()
    print("Six-step parking test complete!")
