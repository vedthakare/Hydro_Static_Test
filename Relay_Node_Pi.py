#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import String, Bool  # Import both String and Bool message types

# Define GPIO pins for each relay (edit if needed)
RELAY_PINS = [6, 13, 19, 26]

# Global variable to track emergency stop state
emergency_active = False

def setup_gpio():
    GPIO.setmode(GPIO.BCM)  # Use BCM numbering
    for pin in RELAY_PINS:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.HIGH)  # Start all relays OFF

def emergency_stop_callback(data):
    global emergency_active
    emergency_active = data.data
    
    if emergency_active:
        # In emergency stop mode, set all relays to HIGH (1,1,1,1)
        for pin in RELAY_PINS:
            GPIO.output(pin, GPIO.HIGH)
        rospy.logwarn("EMERGENCY STOP ACTIVATED - All gates turned off (1,1,1,1)")
    else:
        rospy.loginfo("Emergency stop deactivated - Resuming normal operation")

def relay_callback(data):
    # If emergency stop is active, ignore relay state changes
    global emergency_active
    if emergency_active:
        rospy.loginfo("Ignoring relay state change due to active emergency stop")
        return
    
    # Message should look like: "0,1,0,1"
    try:
        states = list(map(int, data.data.strip().split(',')))
        if len(states) != 4:
            rospy.logwarn("Expected 4 values, got: %s", data.data)
            return
        # Set each relay ON (Low) or OFF (High)
        for i in range(4):
            GPIO.output(RELAY_PINS[i], GPIO.HIGH if states[i] else GPIO.LOW)
        rospy.loginfo("Relays set to: %s", states)
    except Exception as e:
        rospy.logerr("Invalid input: %s", data.data)
        rospy.logerr("Exception: %s", str(e))

def main():
    rospy.init_node('relay_control_node')
    setup_gpio()
    
    # Subscribe to both emergency stop and relay states topics
    rospy.Subscriber('emergency_stop', Bool, emergency_stop_callback)
    rospy.Subscriber('relay_states', String, relay_callback)
    
    rospy.loginfo("Relay node ready and listening for relay_states and emergency_stop...")
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
        rospy.loginfo("GPIO cleaned up.")

if __name__ == '__main__':
    main()