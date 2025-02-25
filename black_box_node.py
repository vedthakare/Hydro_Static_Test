#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
from datetime import datetime
import csv
import os

class BlackBoxLogger:
    def __init__(self):
        # Create filename with current date and time
        start_time = datetime.now()
        self.start_timestamp = start_time
        filename = f"data_log_{start_time.strftime('%Y%m%d_%H%M%S')}.csv"
        self.filename = filename
        
        # Create CSV file with headers
        with open(self.filename, "w", newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Relative_Time_Seconds", "Topic", "Data"])
        
    def log_data(self, topic_name, msg):
        # Calculate relative time in seconds since start
        relative_time = (datetime.now() - self.start_timestamp).total_seconds()
        with open(self.filename, "a", newline='') as f:
            writer = csv.writer(f)
            writer.writerow([f"{relative_time:.6f}", topic_name, msg.data])

def main():
    rospy.init_node('black_box_node')
    
    # Initialize logger
    logger = BlackBoxLogger()
    
    # Add subscribers - easily extendable for more topics
    topics = [
        ('local_sensor_data', Int32),
        ('local_sensor_data_2', Int32)
        # Add more topics here as needed: ('topic_name', msg_type)
    ]
    
    # Create subscribers for each topic
    subscribers = [
        rospy.Subscriber(
            topic_name,
            msg_type,
            lambda msg, tn=topic_name: logger.log_data(tn, msg)
        )
        for topic_name, msg_type in topics
    ]
    
    rospy.spin()

if __name__ == '__main__':
    main()
