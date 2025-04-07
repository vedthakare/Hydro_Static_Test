#!/usr/bin/env python3
import rospy
import serial
import time
from std_msgs.msg import Float32

def calculate_P(V_out, V_supply, P_max, P_min):
    return ((V_out - 0.1 * V_supply) * (P_max - P_min)) / (0.8 * V_supply)

# Serial Setup
ser = serial.Serial('/dev/ttyS0', 115200)

rospy.init_node('serial_node')
pubs = {
    'pt1_v': rospy.Publisher('pt1_v', Float32, queue_size=10),
    'pt2_v': rospy.Publisher('pt2_v', Float32, queue_size=10),
    'tc1_v': rospy.Publisher('tc1_v', Float32, queue_size=10),
    'tc2_v': rospy.Publisher('tc2_v', Float32, queue_size=10),
    'loadcell': rospy.Publisher('loadcell', Float32, queue_size=10)
}

def read_serial():
    time.sleep(0.5)
    while not rospy.is_shutdown():
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                values = line.split(',')
                if len(values) == 9:
                    raw_pt1 = int(values[0])
                    pt1_v = float(values[1])
                    raw_pt2 = int(values[2])
                    pt2_v = float(values[3])
                    raw_tc1 = int(values[4])
                    tc1_v = float(values[5])
                    raw_tc2 = int(values[6])
                    tc2_v = float(values[7])
                    loadcell = int(values[8])


                    # Publish
                    pubs['pt1_v'].publish(pt1_v)
                    pubs['pt2_v'].publish(pt2_v)
                    pubs['tc1_v'].publish(tc1_v)
                    pubs['tc2_v'].publish(tc2_v)
                    pubs['loadcell'].publish(loadcell)
        except (UnicodeDecodeError, ValueError):
            rospy.logwarn("Serial parsing error")
        except serial.SerialException:
            rospy.logerr("Serial port error")
            break
        except rospy.ROSInterruptException:
            break

if __name__ == '__main__':
    try:
        read_serial()
    except:
        print('didnt work')
