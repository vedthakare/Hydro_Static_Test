import rospy
from std_msgs.msg import Int32

def log_data(msg):
    with open("data_log.txt", "a") as f:
        f.write(f"{msg.data}\n")

rospy.init_node('black_box_node')
rospy.Subscriber('remote_sensor_data', Int32, log_data)
rospy.spin();