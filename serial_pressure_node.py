#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import Float32
import time

ser = serial.Serial('/dev/ttyS0', 115200)
rospy.init_node('serial_node')
pub = rospy.Publisher('voltage_pressure_data', Float32, queue_size=10)
pub2 = rospy.Publisher('voltage_pressure_data_2', Float32, queue_size=10)

def read_serial():
    
    while not rospy.is_shutdown():
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                    
                values = line.split(',')
                if len(values) == 2:
                    sensor1_value = float(values[0])
                    sensor2_value = float(values[1])
                    pub.publish(sensor1_value)
                    pub2.publish(sensor2_value)
        except UnicodeDecodeError:
            rospy.logwarn("Serial decode error")
        except ValueError:
            rospy.logwarn("Invalid float conversion")
        except serial.SerialException:
            rospy.logerr("Serial port error")
            break
        except rospy.ROSInterruptException:
            break
        time.sleep(0.001)#maybe remove

if __name__ == '__main__':
    read_serial()
