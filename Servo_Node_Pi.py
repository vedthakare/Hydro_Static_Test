#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

# Servo configuration
SERVO_PIN = 12      # GPIO 12 (BCM numbering)
FREQ = 80          # 50 Hz standard for servos

# Define servo parameters (adjust these values as needed)
SERVO_MIN_ANGLE = 0
SERVO_MAX_ANGLE = 270
PULSE_MIN_MS = 0.5     # Pulse width at min angle (ms)
PULSE_MAX_MS = 2.0     # Pulse width at max angle (ms)
PWM_PERIOD_MS = 1000.0 / FREQ

SERVO_MIN_DUTY = (PULSE_MIN_MS / PWM_PERIOD_MS) * 100.0
SERVO_MAX_DUTY = (PULSE_MAX_MS / PWM_PERIOD_MS) * 100.0

DEAD_BAND_US = 2
DEAD_BAND = (DEAD_BAND_US / (PWM_PERIOD_MS * 1000)) * 100
# Anti-jitter parameters
SMOOTHING_FACTOR = 0.2     # Lower values mean more smoothing (0.0 to 1.0)
MIN_ANGLE_CHANGE = 1     # Ignore changes smaller than this value
UPDATE_RATE = 40           # Hz - frequency to update servo position
class ServoController:
    def __init__(self):
        self.current_angle = 0
        self.target_angle = 0
        self.last_duty = 0
        self.last_update_time = 0
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(SERVO_PIN, FREQ)
        self.pwm.start(0)
        
    def angle_to_duty_cycle(self, angle):
        """Convert the input angle to the corresponding PWM duty cycle."""
        # Clamp the angle to the allowed range
        angle = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, angle))
        # Calculate the duty cycle using linear interpolation
        angle_range = SERVO_MAX_ANGLE - SERVO_MIN_ANGLE
        duty_range = SERVO_MAX_DUTY - SERVO_MIN_DUTY
        duty = SERVO_MIN_DUTY + ((angle - SERVO_MIN_ANGLE) / angle_range) * duty_range
        return duty
    
    def angle_callback(self, msg):
        """Handle incoming angle messages"""
        # Extract the incoming angle
        self.target_angle = msg.data
        rospy.logdebug(f"Received target angle: {self.target_angle:.1f}°")
    
    def update_servo(self, event=None):
        """Update servo position with smoothing"""
        # If the change is too small, ignore it
        if abs(self.target_angle - self.current_angle) < MIN_ANGLE_CHANGE:
            return
            
        # Apply exponential smoothing to reduce jitter
        self.current_angle = (SMOOTHING_FACTOR * self.target_angle + 
                              (1 - SMOOTHING_FACTOR) * self.current_angle)
        
        # Calculate duty cycle
        duty = self.angle_to_duty_cycle(self.current_angle)
        
        # Apply dead band to avoid tiny PWM changes
        if abs(duty - self.last_duty) > DEAD_BAND:
            self.pwm.ChangeDutyCycle(duty)
            self.last_duty = duty
            rospy.logdebug(f"Updated angle: {self.current_angle:.1f}° → Duty: {duty:.2f}%")
        
    def shutdown(self):
        """Clean up GPIO on shutdown"""
        self.pwm.stop()
        GPIO.cleanup()
        rospy.loginfo("PWM stopped, GPIO cleaned.")

def main():
    rospy.init_node('servo_control_node')
    controller = ServoController()
    
    # Create a subscriber with queue_size=1 to drop old messages
    rospy.Subscriber('servo_angle', Float32, controller.angle_callback, queue_size=1)
    
    # Create a timer to update the servo position at a fixed rate
    timer = rospy.Timer(rospy.Duration(1.0/UPDATE_RATE), controller.update_servo)
    
    rospy.loginfo(f"Servo controller running at {UPDATE_RATE}Hz update rate")
    rospy.loginfo("Listening on /servo_angle topic...")
    
    rospy.on_shutdown(controller.shutdown)
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()