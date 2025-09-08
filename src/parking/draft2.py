from gpiozero import DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory
from time import time, sleep
import pigpio

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

# Plan C Parking State Machine
class PlanCParkingMachine:
    def __init__(self):
        self.state = "STEP1_TURN_90"  # STEP1_TURN_90, STEP2_FORWARD_5SEC, STEP3_LEFT_PARK, 
                                      # STEP4_RIGHT_PARK, STEP5_FINAL_ADJUST, PARKED
        self.step_start_time = None
        
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
        print(f"🔧 Adjusting position - F:{sensors['front']:.1f} B:{sensors['back']:.1f} L:{sensors['left']:.1f} R:{sensors['right']:.1f}")
        
        moved = False
        
        # Priority 1: Front/Back distance control
        if sensors['front'] <= 8:
            print("⬅️ Front too close - moving backward")
            pwm.set_servo_pulsewidth(servo_pin, 1500)  # Straight
            motorSpeed(-40)  # Move backward
            sleep(0.5)
            motorSpeed(0)
            moved = True
            
        elif sensors['back'] <= 8:
            print("➡️ Back too close - moving forward") 
            pwm.set_servo_pulsewidth(servo_pin, 1500)  # Straight
            motorSpeed(40)  # Move forward
            sleep(0.5)
            motorSpeed(0)
            moved = True
        
        # Priority 2: Side distance control (if front/back are OK)
        if not moved:
            if sensors['left'] <= 5:
                print("↗️ Left side too close - steering right and adjusting")
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
                print("↖️ Right side too close - steering left and adjusting")
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
        
    def execute_parking(self, current_time):
        sensors = self.get_sensor_readings()
        
        print(f"📍 Step: {self.state}")
        print(f"📏 Sensors - F:{sensors['front']:.1f} B:{sensors['back']:.1f} L:{sensors['left']:.1f} R:{sensors['right']:.1f}")
        
        if self.state == "STEP1_TURN_90":
            # Step 1: Turn 90 degrees using front servo
            if self.step_start_time is None:
                self.step_start_time = current_time
                print("🔄 Starting 90-degree turn")
                
            elapsed = current_time - self.step_start_time
            
            if elapsed < 3.0:  # Turn for 3 seconds (90 degrees)
                pwm.set_servo_pulsewidth(servo_pin, 1650)  # Right turn (gentle for PLA wheels)
                motorSpeed(35)  # Slow turning speed
                print(f"🔄 Turning 90 degrees... {elapsed:.1f}s")
            else:
                motorSpeed(0)
                pwm.set_servo_pulsewidth(servo_pin, 1500)  # Center servo
                self.state = "STEP2_FORWARD_5SEC"
                self.step_start_time = current_time
                print("✅ STEP 1 COMPLETE: 90-degree turn finished")
                
        elif self.state == "STEP2_FORWARD_5SEC":
            # Step 2: Move forward for 5 seconds and stop
            if self.step_start_time is None:
                self.step_start_time = current_time
                print("⬆️ Moving forward for 5 seconds")
                
            elapsed = current_time - self.step_start_time
            
            if elapsed < 5.0:  # Move forward for 5 seconds
                pwm.set_servo_pulsewidth(servo_pin, 1500)  # Straight
                motorSpeed(50)  # Forward
                print(f"⬆️ Moving forward... {elapsed:.1f}s")
            else:
                motorSpeed(0)
                self.state = "STEP3_LEFT_PARK"
                self.step_start_time = current_time
                print("✅ STEP 2 COMPLETE: Forward movement finished")
                
        elif self.state == "STEP3_LEFT_PARK":
            # Step 3: Turn servo left (25 degrees), move backward into parking lot
            print("🏁 STEP 3: Left steering parking attempt")
            pwm.set_servo_pulsewidth(servo_pin, 1350)  # Turn left (25 degrees)
            
            # Check if we can still reverse safely
            if sensors['back'] > 10:  # Safe to reverse
                motorSpeed(-45)  # Reverse into parking space
                print("⬇️ Reversing into parking space with left steer")
            else:
                motorSpeed(0)
                print("🛑 Too close to back obstacle, stopping reverse")
            
            # Check if adjustment needed or if we should move to next step
            if self.check_parking_complete(sensors):
                motorSpeed(0)
                pwm.set_servo_pulsewidth(servo_pin, 1500)
                self.state = "PARKED"
                print("🎉 PARKING COMPLETE after left steering!")
            elif not self.adjust_position_in_parking(sensors):
                # If no adjustment was needed, move to right steering attempt
                self.state = "STEP4_RIGHT_PARK"
                self.step_start_time = current_time
                print("➡️ Moving to right steering attempt")
                
        elif self.state == "STEP4_RIGHT_PARK":
            # Step 4: Turn servo right (25 degrees), move backward into parking lot  
            print("🏁 STEP 4: Right steering parking attempt")
            pwm.set_servo_pulsewidth(servo_pin, 1650)  # Turn right (25 degrees)
            
            # Check if we can still reverse safely
            if sensors['back'] > 10:  # Safe to reverse
                motorSpeed(-45)  # Reverse into parking space
                print("⬇️ Reversing into parking space with right steer")
            else:
                motorSpeed(0)
                print("🛑 Too close to back obstacle, stopping reverse")
            
            # Check if adjustment needed or if we should move to final adjustment
            if self.check_parking_complete(sensors):
                motorSpeed(0)
                pwm.set_servo_pulsewidth(servo_pin, 1500)
                self.state = "PARKED"
                print("🎉 PARKING COMPLETE after right steering!")
            elif not self.adjust_position_in_parking(sensors):
                # Move to final adjustment phase
                self.state = "STEP5_FINAL_ADJUST"
                self.step_start_time = current_time
                print("🔧 Moving to final adjustment phase")
                
        elif self.state == "STEP5_FINAL_ADJUST":
            # Step 5: Final position adjustments until all thresholds satisfied
            print("🔧 STEP 5: Final position adjustment")
            
            if self.check_parking_complete(sensors):
                motorSpeed(0)
                pwm.set_servo_pulsewidth(servo_pin, 1500)
                self.state = "PARKED"
                print("🎉 PARKING COMPLETE after final adjustments!")
                return True
            else:
                # Keep adjusting position
                self.adjust_position_in_parking(sensors)
                
        elif self.state == "PARKED":
            # Final state - stay parked
            motorSpeed(0)
            pwm.set_servo_pulsewidth(servo_pin, 1500)
            print("🅿️ SUCCESSFULLY PARKED!")
            print(f"📏 Final position - F:{sensors['front']:.1f} B:{sensors['back']:.1f} L:{sensors['left']:.1f} R:{sensors['right']:.1f}")
            return True
            
        return False  # Parking still in progress

# Initialize Plan C parking system
parking_system = PlanCParkingMachine()

print("=" * 50)
print("🚗 PLAN C - ULTRASONIC-ONLY PARKING SYSTEM")
print("=" * 50)
print("1️⃣ Turn 90 degrees")
print("2️⃣ Move forward 5 seconds")  
print("3️⃣ Left steering + reverse parking")
print("4️⃣ Right steering + reverse parking")
print("5️⃣ Final position adjustments")
print("🎯 Target: F>8cm, B>8cm, L>5cm, R>5cm")
print("Press Ctrl+C to emergency stop")
print("=" * 50)

try:
    while True:
        current_time = time()

        # Execute Plan C parking sequence
        parking_complete = parking_system.execute_parking(current_time)
        
        # Auto-exit when parking is complete
        if parking_complete:
            print("🎉 PLAN C PARKING SEQUENCE COMPLETED SUCCESSFULLY! 🎉")
            print("Waiting 3 seconds before exit...")
            sleep(3)
            break
            
        sleep(0.1)  # Small delay for stability

except KeyboardInterrupt:
    print("\n🛑 EMERGENCY STOP - Manual interrupt")

finally:
    print("🔧 Stopping robot and cleaning up...")
    motorSpeed(0)
    pwm.set_servo_pulsewidth(servo_pin, 1500)  # Center servo
    sleep(0.5)
    pwm.set_servo_pulsewidth(servo_pin, 0)     # Turn off servo
    pi.stop()
    print("✅ Plan C parking test complete!")
