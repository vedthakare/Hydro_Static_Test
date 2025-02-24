#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def publish_video():
    rospy.init_node('video_publisher', anonymous=True)
    pub = rospy.Publisher('/video_stream', Image, queue_size=10)
    bridge = CvBridge()
    cap = cv2.VideoCapture(0) # Change if using CSI camera

    if not cap.isOpened():
        rospy.logerr("Failed to open camera")
        return

    rate = rospy.Rate(30)  # 30 FPS

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to read frame")
            continue
        
        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        pub.publish(msg)
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        publish_video()
    except rospy.ROSInterruptException:
        pass
