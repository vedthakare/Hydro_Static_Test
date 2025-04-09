#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO

# Servo configuration
SERVO_PIN = 12      # GPIO 12 (BCM numbering)
FREQ = 50           # 50 Hz standard for servos

# Define servo parameters (adjust these values as needed)
SERVO_MIN_ANGLE = 0
SERVO_MAX_ANGLE = 270
SERVO_MIN_DUTY = 3.0   # Duty cycle (%) corresponding to SERVO_MIN_ANGLE
SERVO_MAX_DUTY = 12.0  # Duty cycle (%) corresponding to SERVO_MAX_ANGLE

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    pwm = GPIO.PWM(SERVO_PIN, FREQ)
    pwm.start(0)  # Start with a 0% duty cycle
    return pwm

def angle_to_duty_cycle(angle):
    """
    Convert the input angle to the corresponding PWM duty cycle.
    The conversion is linear based on the predefined minimum and maximum values.
    """
    # Clamp the angle to the allowed range.
    angle = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, angle))
    # Calculate the duty cycle using linear interpolation.
    angle_range = SERVO_MAX_ANGLE - SERVO_MIN_ANGLE
    duty_range = SERVO_MAX_DUTY - SERVO_MIN_DUTY
    duty = SERVO_MIN_DUTY + ((angle - SERVO_MIN_ANGLE) / angle_range) * duty_range
    return duty

def angle_callback(msg):
    # Extract the incoming angle.
    angle = msg.data
    # Compute the duty cycle for the given angle.
    duty = angle_to_duty_cycle(angle)
    rospy.loginfo("Received angle: {:.1f}° → Calculated duty cycle: {:.2f}%".format(angle, duty))
    # Update the PWM duty cycle accordingly.
    pwm.ChangeDutyCycle(duty)

def main():
    global pwm
    rospy.init_node('servo_control_node')
    pwm = setup_gpio()
    rospy.Subscriber('servo_angle', Float32, angle_callback)
    rospy.loginfo("Listening on /servo_angle topic...")
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        pwm.stop()
        GPIO.cleanup()
        rospy.loginfo("PWM stopped, GPIO cleaned.")

if __name__ == '__main__':
    main()