#!/usr/bin/env python3
import rospy
import cv2
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class VideoSubscriber:
    def __init__(self):
        rospy.init_node('video_subscriber', anonymous=True)
        rospy.Subscriber('/video_stream', Image, self.callback)
        self.bridge = CvBridge()
        self.output_path = "video_output.avi"
        self.video_writer = None

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        # Initialize video writer if not already initialized
        if self.video_writer is None:
            height, width, _ = frame.shape
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.video_writer = cv2.VideoWriter(self.output_path, fourcc, 30, (width, height))

        self.video_writer.write(frame)
        cv2.imshow("Live Video", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("User Exit")

    def run(self):
        rospy.spin()
        if self.video_writer:
            self.video_writer.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = VideoSubscriber()
        node.run()
    except rospy.ROSInterruptException:
        pass
