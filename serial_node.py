#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import Int32
import time

ser = serial.Serial('/dev/ttyUSB0', 115200)
rospy.init_node('serial_node')
pub = rospy.Publisher('local_sensor_data', Int32, queue_size=10)
pub2 = rospy.Publisher('local_sensor_data_2', Int32, queue_size=10)

def read_serial():
    while not rospy.is_shutdown():
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                values = line.split(',')  # Split the string at comma
                if len(values) == 2:  # Make sure we got both values
                    sensor1_value = int(values[0])
                    sensor2_value = int(values[1])
                    # Now you can use sensor1_value and sensor2_value
                    pub.publish(sensor1_value)
                    pub2.publish(sensor2_value)
        except UnicodeDecodeError:
            rospy.logwarn("Serial decode error")
        except ValueError:
            rospy.logwarn("Invalid integer conversion")
        except serial.SerialException:
            rospy.logerr("Serial port error")
            break
        except rospy.ROSInterruptException:
            break
        time.sleep(0.001)#maybe remove

if __name__ == '__main__':
    read_serial()
