#!/usr/bin/env python3

# Import ROS Python library and message types
import rospy
from std_msgs.msg import String, Float32
import time

def ignition_sequence(wait1,wait2,wait3,wait4):
    # Initialize the ROS node
    rospy.init_node('ignition_sequence_node')

    # Publishers for relay state and two servo angles
    pub_Ignition = rospy.Publisher("/relay_states", String, queue_size=10)
    pub_servo_1 = rospy.Publisher("/servo_angle", Float32, queue_size=3)
    pub_servo_2 = rospy.Publisher("/servo_angle_2", Float32, queue_size=3)

    # Simple password protection for safety
    correct_password = 'ignite123'
    password = input('Enter password to start ignition: ')
    
    if password != correct_password:
        print('❌ Incorrect password. Aborting.')
        return

    # Countdown before ignition sequence begins
    print('✅ Password accepted. Starting ignition countdown...')
    for i in range(5, 0, -1):
        print(f'{i}...')
        time.sleep(1)

    print('🚀 Ignition sequence initiated!')

    rospy.sleep(1)

    # === STAGE 1: Open valve 1 ===
    pub_servo_1.publish(20)  # Open valve 1 fully
    print(' Published to /servo_angle to Open Valve 1')
    rospy.sleep(wait1)#1

    # === STAGE 2: Intiate the ignition ===
    pub_Ignition.publish('1,1,1,0') # Turn on relay 4 for ignition
    print(' Published to /relay_states to Turn on Ignition')
    rospy.sleep(wait2)

    # === STAGE 3: Open Valve 2 / Intiate Hot Fire ===
    pub_servo_2.publish(15)  # Open valve 2 Fully
    print(' Published to /servo_angle_2 to Open Valve 2')
    rospy.sleep(wait3)#6

    # === STAGE 4: Fully Close both valves and turn off ignition ===
    pub_Ignition.publish('1,1,1,1') # Turn on relay 4 for ignition
    print(' Published to /relay_states to Turn off Ignition')
    pub_servo_1.publish(100)  # Open valve 1 fully closed
    print(' Published to /servo_angle to Close Valve 1')
    rospy.sleep(wait4)#1
    pub_servo_2.publish(95)  # Open valve 2 fully closed
    print(' Published to /servo_angle_2 to Close Valve 2')

    print(' Sequence complete.')

# Run the ignition sequence if this script is run directly
if __name__ == '__main__':
    wait1=1
    wait2=0.2
    wait3=6
    wait4=1
    try:
        ignition_sequence(wait1=wait1,wait2=wait2,wait3=wait3,wait4=wait4)
    except rospy.ROSInterruptException:
        pass  # Gracefully handle node shutdown
