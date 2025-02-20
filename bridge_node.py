#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

def bridge_callback(msg):
    bridge_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('bridge_node')
    rospy.Subscriber('local_sensor_data', Int32, bridge_callback)
    bridge_pub = rospy.Publisher('remote_sensor_data', Int32, queue_size=10)
    rospy.spin() 