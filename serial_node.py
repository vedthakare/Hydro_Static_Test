import rospy
import serial
from std_msgs.msg import Int32
import time

ser = serial.Serial('/dev/ttyUSB0', 115200)
rospy.init_node('serial_node')
pub = rospy.Publisher('local_sensor_data', Int32, queue_size=10)

def read_serial():
    while not rospy.is_shutdown():
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                pub.publish(int(line))
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
