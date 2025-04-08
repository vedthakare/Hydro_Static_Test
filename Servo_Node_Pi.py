#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
from time import sleep

SERVO_PIN = 12  # Physical pin 32 = GPIO 12 in BOARD mode
FREQ = 50       # Standard servo PWM frequency in Hz

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    pwm = GPIO.PWM(SERVO_PIN, FREQ)
    pwm.start(0)  # Start PWM at 0% duty cycle
    return pwm

def angle_to_duty_cycle(angle):
    # Clamp angle to [0, 270]
    angle = max(0, min(270, angle))
    # Map 0–270° → 3–12% duty cycle
    return 3 + (angle / 270.0) * 9.0

def angle_callback(msg):
    angle = msg.data
    duty = angle_to_duty_cycle(angle)
    pwm.ChangeDutyCycle(duty)
    rospy.loginfo(f"Angle: {angle:.1f}° → Duty: {duty:.2f}%")
    sleep(0.3)
    pwm.ChangeDutyCycle(0)  # Stop signal to prevent jitter

def main():
    global pwm
    rospy.init_node('servo_control_node')
    pwm = setup_gpio()
    rospy.Subscriber('servo_angle', Float32, angle_callback)
    rospy.loginfo("Listening on /servo_angle...")
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        pwm.stop()
        GPIO.cleanup()
        rospy.loginfo("PWM stopped, GPIO cleaned.")

if __name__ == '__main__':
    #
    main()
