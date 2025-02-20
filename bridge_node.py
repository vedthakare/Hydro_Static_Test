#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

def bridge_callback1(msg):
    bridge_pub.publish(msg)
def bridge_callback2(msg):
    bridge_pub_2.publish(msg)
if __name__ == '__main__':
    rospy.init_node('bridge_node')
    rospy.Subscriber('local_sensor_data', Int32, bridge_callback1)
    rospy.Subscriber('local_sensor_data_2', Int32, bridge_callback2)
    bridge_pub = rospy.Publisher('remote_sensor_data', Int32, queue_size=10)
    bridge_pub_2 = rospy.Publisher('remote_sensor_data_2', Int32, queue_size=10)
    rospy.spin() 