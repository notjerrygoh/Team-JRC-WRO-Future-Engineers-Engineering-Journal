from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
from time import time, sleep
import pigpio
import cv2
import numpy as np
from picamera2 import Picamera2

pi = pigpio.pi()

# Initialize camera
cam = Picamera2()
cam.preview_configuration.main.size = (640, 480)
cam.preview_configuration.main.format = "RGB888"
cam.preview_configuration.align()
cam.configure("preview")
cam.start()

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

# Motors
M1A = 23
M1B = 24
pi.set_mode(M1A, pigpio.OUTPUT)
pi.set_mode(M1B, pigpio.OUTPUT)

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

# Orange line detection function
def detect_orange_line(frame):
    """Detect orange lines in the frame and return count"""
    # Convert to HSV for better color detection
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Orange color range (adjust these values based on your orange lines)
    lower_orange = np.array([10, 100, 100])  # Lower bound for orange in HSV
    upper_orange = np.array([25, 255, 255])  # Upper bound for orange in HSV
    
    # Create mask for orange color
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Filter contours that look like lines (horizontal, certain size)
    line_contours = []
    for contour in contours:
        # Get bounding rectangle
        x, y, w, h = cv2.boundingRect(contour)
        
        # Check if it's line-like (wider than it is tall, minimum size)
        if w > h * 2 and w > 50 and h > 5:
            line_contours.append(contour)
    
    return len(line_contours), mask

# Game Field Parking State Machine
class GameFieldParkingMachine:
    def __init__(self):
        self.state = "NAVIGATE_TO_12TH_LINE"  # NAVIGATE_TO_12TH_LINE, CENTER_BETWEEN_WALLS, 
                                              # TURN_TO_PARKING, REVERSE_PARK, FINE_TUNE, PARKED
        self.orange_line_count = 11  # START FROM 11 FOR TESTING
        self.step_start_time = None
        self.last_line_detection_time = 0
        self.line_detection_cooldown = 2.0  # Prevent double counting
        
    def get_sensor_readings(self):
        """Get all ultrasonic sensor readings in cm"""
        try:
            readings = {
                'front': frontultrasonic.distance * 100,
                'back': backultrasonic.distance * 100,
                'left': leftultrasonic.distance * 100,
                'right': rightultrasonic.distance * 100
            }
            # Cap readings at 200cm for stability
            for key in readings:
                if readings[key] > 200:
                    readings[key] = 200
            return readings
        except:
            # Return safe values if sensor read fails
            return {'front': 200, 'back': 200, 'left': 200, 'right': 200}
    
    def adjust_position_in_parking(self, sensors):
        """Handle position adjustment when in parking space"""
        print(f"Adjusting position - F:{sensors['front']:.1f} B:{sensors['back']:.1f} L:{sensors['left']:.1f} R:{sensors['right']:.1f}")
        
        moved = False
        
        # Front/Back distance control
        if sensors['front'] <= 8:
            print("Front too close - moving backward")
            pwm.set_servo_pulsewidth(servo_pin, 1500)  # Straight
            motorSpeed(-40)  # Move backward
            sleep(0.5)
            motorSpeed(0)
            moved = True
            
        elif sensors['back'] <= 8:
            print("Back too close - moving forward") 
            pwm.set_servo_pulsewidth(servo_pin, 1500)  # Straight
            motorSpeed(40)  # Move forward
            sleep(0.5)
            motorSpeed(0)
            moved = True
        
        # Side distance control (if front/back are OK)
        if not moved:
            if sensors['left'] <= 5:
                print("Left side too close - steering right and adjusting")
                pwm.set_servo_pulsewidth(servo_pin, 1650)  # Turn right (25 degrees)
                # Choose direction based on which has more space
                if sensors['front'] > sensors['back']:
                    motorSpeed(40)  # Move forward
                    print("Moving forward with right steer")
                else:
                    motorSpeed(-40)  # Move backward
                    print("Moving backward with right steer")
                sleep(1.0)  # 1 second movement
                motorSpeed(0)
                moved = True
                
            elif sensors['right'] <= 5:
                print("Right side too close - steering left and adjusting")
                pwm.set_servo_pulsewidth(servo_pin, 1350)  # Turn left (25 degrees)
                # Choose direction based on which has more space
                if sensors['front'] > sensors['back']:
                    motorSpeed(40)  # Move forward
                    print("Moving forward with left steer")
                else:
                    motorSpeed(-40)  # Move backward
                    print("Moving backward with left steer")
                sleep(1.0)  # 1 second movement
                motorSpeed(0)
                moved = True
        
        # Return servo to center after adjustment
        pwm.set_servo_pulsewidth(servo_pin, 1500)
        sleep(0.2)
        
        return moved
    
    def check_parking_complete(self, sensors):
        """Check if parking thresholds are satisfied"""
        front_back_ok = sensors['front'] > 8 and sensors['back'] > 8
        sides_ok = sensors['left'] > 5 and sensors['right'] > 5
        
        return front_back_ok and sides_ok
        
    def execute_parking(self, frame, current_time):
        sensors = self.get_sensor_readings()
        
        print(f"Step: {self.state}")
        print(f"Orange lines detected: {self.orange_line_count}")
        print(f"Sensors - F:{sensors['front']:.1f} B:{sensors['back']:.1f} L:{sensors['left']:.1f} R:{sensors['right']:.1f}")
        
        if self.state == "NAVIGATE_TO_12TH_LINE":
            # Navigate forward while counting orange lines
            line_count, orange_mask = detect_orange_line(frame)
            
            # Check if we detected new lines (with cooldown to prevent double counting)
            if line_count > 0 and (current_time - self.last_line_detection_time) > self.line_detection_cooldown:
                self.orange_line_count += line_count
                self.last_line_detection_time = current_time
                print(f"NEW ORANGE LINE DETECTED! Total count: {self.orange_line_count}")
            
            # Navigate forward with obstacle avoidance
            if sensors['front'] > 15:  # Safe to move forward
                pwm.set_servo_pulsewidth(servo_pin, 1500)  # Straight
                motorSpeed(50)  # Move forward
                print(f"Navigating to 12th line... Current: {self.orange_line_count}")
            else:
                # Stop if obstacle ahead
                motorSpeed(0)
                print("Obstacle detected, stopping navigation")
            
            # Check if we've reached the 12th line
            if self.orange_line_count >= 12:
                motorSpeed(0)
                self.state = "CENTER_BETWEEN_WALLS"
                self.step_start_time = current_time
                print("INITIATING PARKING SYSTEM.")
            
            # Show orange detection mask for debugging
            cv2.imshow("Orange Line Detection", orange_mask)
                
        elif self.state == "CENTER_BETWEEN_WALLS":
            # Center robot between left and right walls
            left_dist = sensors['left']
            right_dist = sensors['right']
            
            print(f"Centering - Left: {left_dist:.1f}cm, Right: {right_dist:.1f}cm")
            
            # Calculate difference
            diff = left_dist - right_dist
            
            if abs(diff) > 3:  # Not centered enough
                if diff > 0:  # More space on left, move left
                    pwm.set_servo_pulsewidth(servo_pin, 1350)  # Turn left
                    print("Moving left to center")
                else:  # More space on right, move right
                    pwm.set_servo_pulsewidth(servo_pin, 1650)  # Turn right
                    print("Moving right to center")
                motorSpeed(30)  # Slow movement for precision
            else:
                # Centered! Move to next phase
                motorSpeed(0)
                pwm.set_servo_pulsewidth(servo_pin, 1500)  # Straight
                self.state = "TURN_TO_PARKING"
                self.step_start_time = current_time
                print("CENTERED! Moving to parking turn phase")
                
        elif self.state == "TURN_TO_PARKING":
            # Turn toward the parking lot (side with more space)
            left_dist = sensors['left']
            right_dist = sensors['right']
            
            if self.step_start_time is None:
                self.step_start_time = current_time
                
            elapsed = current_time - self.step_start_time
            
            # Determine which side has more space
            turn_direction = "left" if left_dist > right_dist else "right"
            
            if elapsed < 2.0:  # Turn for 2 seconds
                if turn_direction == "left":
                    pwm.set_servo_pulsewidth(servo_pin, 1350)  # Turn left
                    print(f"Turning left toward parking (L:{left_dist:.1f} > R:{right_dist:.1f})")
                else:
                    pwm.set_servo_pulsewidth(servo_pin, 1650)  # Turn right
                    print(f"Turning right toward parking (R:{right_dist:.1f} > L:{left_dist:.1f})")
                motorSpeed(35)  # Turning speed
            else:
                motorSpeed(0)
                pwm.set_servo_pulsewidth(servo_pin, 1500)  # Center
                self.state = "REVERSE_PARK"
                self.step_start_time = current_time
                print("TURN COMPLETE! Starting reverse parking")
                
        elif self.state == "REVERSE_PARK":
            # Reverse into the parking space
            print("Reverse parking into space")
            
            # Check if we can still reverse safely
            if sensors['back'] > 10:  # Safe to reverse
                pwm.set_servo_pulsewidth(servo_pin, 1500)  # Straight
                motorSpeed(-45)  # Reverse into parking space
                print("Reversing into parking space")
            else:
                motorSpeed(0)
                self.state = "FINE_TUNE"
                self.step_start_time = current_time
                print("REVERSE COMPLETE! Moving to fine-tuning")
                
        elif self.state == "FINE_TUNE":
            # Fine-tune position until all thresholds satisfied
            print("Fine-tuning position")
            
            if self.check_parking_complete(sensors):
                motorSpeed(0)
                pwm.set_servo_pulsewidth(servo_pin, 1500)
                self.state = "PARKED"
                print("PARKING COMPLETE!")
                return True
            else:
                # Keep adjusting position
                self.adjust_position_in_parking(sensors)
                
        elif self.state == "PARKED":
            # Final state - stay parked
            motorSpeed(0)
            pwm.set_servo_pulsewidth(servo_pin, 1500)
            print("SUCCESSFULLY PARKED!")
            print(f"Final position - F:{sensors['front']:.1f} B:{sensors['back']:.1f} L:{sensors['left']:.1f} R:{sensors['right']:.1f}")
            return True
            
        return False  # Parking still in progress

# Initialize Parking system
parking_system = GameFieldParkingMachine()

print("=" * 60)
print("PARKING IN PROGRESS")
print("=" * 60)
print("1. Navigate until 12th orange line detected")
print("2️. Center robot between walls")  
print("3️. Turn toward parking lot (side with more space)")
print("4️. Reverse park into space")
print("5️. Fine-tune position")
print("🎯 Target: F>8cm, B>8cm, L>5cm, R>5cm")
print("Press Ctrl+C to emergency stop")
print("=" * 60)

try:
    while True:
        frame = cv2.cvtColor(cam.capture_array()[::-1, :, :3], cv2.COLOR_RGB2BGR)
        current_time = time()

        # Execute Game Field parking sequence
        parking_complete = parking_system.execute_parking(frame, current_time)
        
        # Show camera feed for debugging
        cv2.imshow("Game Field Parking - Orange Detection", frame)
        
        # Exit condition
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
        
        # Auto-exit when parking is complete
        if parking_complete:
            print("DONE")
            print("Waiting 3 seconds before exit...")
            sleep(3)
            break
            
        sleep(0.1)  # Small delay for stability

except KeyboardInterrupt:
    print("\nEMERGENCY STOP - Manual interrupt")

finally:
    print("🔧 Stopping robot and cleaning up...")
    motorSpeed(0)
    pwm.set_servo_pulsewidth(servo_pin, 1500)  # Center servo
    sleep(0.5)
    pwm.set_servo_pulsewidth(servo_pin, 0)     # Turn off servo
    cv2.destroyAllWindows()
    cam.stop()
    pi.stop()
    print("Game field parking test complete!")
