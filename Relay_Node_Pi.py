#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO

# Define GPIO pins for each relay (edit if needed)
RELAY_PINS = [6,13,19,26]

def setup_gpio():
    GPIO.setmode(GPIO.BCM)  # Use BCM numbering
    for pin in RELAY_PINS:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)  # Start all relays OFF

def relay_callback(data):
    # Message should look like: "0,1,0,1"
    try:
        states = list(map(int, data.data.strip().split(',')))
        if len(states) != 4:
            rospy.logwarn("Expected 4 values, got: %s", data.data)
            return
        # Set each relay ON (HIGH) or OFF (LOW)
        for i in range(4):
            GPIO.output(RELAY_PINS[i], GPIO.HIGH if states[i] else GPIO.LOW)
        rospy.loginfo("Relays set to: %s", states)
    except:
        rospy.logerr("Invalid input: %s", data.data)

def main():
    rospy.init_node('relay_control_node')
    setup_gpio()
    rospy.Subscriber('relay_states', str, relay_callback)
    rospy.loginfo("Relay node ready and listening...")
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
        rospy.loginfo("GPIO cleaned up.")

if __name__ == '__main__':
    main()
