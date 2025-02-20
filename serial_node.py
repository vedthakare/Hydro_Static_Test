import rospy
import serial
from std_msgs.msg import Int32

ser = serial.Serial('/dev/ttyUSB0', 115200)
rospy.init_node('serial_node')
pub = rospy.Publisher('sensor_data', Int32, queue_size=10)

def read_serial():
    while not rospy.is_shutdown():
        try:
            line = ser.readline().decode('utf-8').strip()
            pub.publish(int(line))
        except:
            continue

if __name__ == '__main__':
    read_serial()