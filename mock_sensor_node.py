#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
import random
import math

rospy.init_node('mock_sensor')
pub = rospy.Publisher('local_sensor_data', Int32, queue_size=10)
rate = rospy.Rate(10)  # 10Hz

while not rospy.is_shutdown():
    # Choose one test pattsern:
    
    # 1. Random values (0-1023)
    value = random.randint(0, 1023)
    
    # 2. Sine wave (simulate analog sensor)
    # value = int(512 * math.sin(rospy.get_time()) + 512)
    
    # 3. Incrementing counter
    # value = int(rospy.get_time() * 10) % 1024
    
    pub.publish(value)
    rate.sleep() 