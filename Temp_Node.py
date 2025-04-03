#!/usr/bin/env python3
import rospy
import serial
import time
from std_msgs.msg import Float32


def calculate_P(V_out, V_supply, P_max, P_min):
    """
    Calculate P using the formula:
    P = ((V_out - 0.1 * V_supply) * (P_max - P_min)) / (0.8 * V_supply)

    Parameters:
    V_out (float): The output voltage.
    V_supply (float): The supply voltage.
    P_max (float): The maximum pressure (or analogous parameter).
    P_min (float): The minimum pressure (or analogous parameter).

    Returns:
    float: The calculated value of P.
    """
    return ((V_out - 0.1 * V_supply) * (P_max - P_min)) / (0.8 * V_supply)


def calculate_T(V_out):
    """
     Temperature = (Vout - 1.25) / 0.005 V
    """
    return ((V_out - 1.25) / (0.005))


# Serial Setup
ser = serial.Serial('/dev/ttyS0', 115200)

rospy.init_node('serial_node')
#pressure data
pub = rospy.Publisher('voltage_pressure_data', Float32, queue_size=10)
pub2 = rospy.Publisher('voltage_pressure_data_2', Float32, queue_size=10)
pub3 = rospy.Publisher('pressure_data', Float32, queue_size=10)
pub4 = rospy.Publisher('pressure_data_2', Float32, queue_size=10)
#temperature data
pub5 = rospy.Publisher('voltage_temp_data', Float32, queue_size=10)
pub6 = rospy.Publisher('voltage_temp_data_2', Float32, queue_size=10)
pub7 = rospy.Publisher('temp_data', Float32, queue_size=10)
pub8 = rospy.Publisher('temp_data_2', Float32, queue_size=10)


def read_serial():
    # Signal to ESP32 to start
    time.sleep(0.5)  # Allow time for ESP32 to start

    while not rospy.is_shutdown():
        try:
            if ser.in_waiting > 0:
                #line should read a statement that contains numbers like #,#,#,#
                line = ser.readline().decode('utf-8').strip()
                values = line.split(',')
                if len(values) == 2:
                    sensor1_pressure_value = float(values[0])
                    sensor2_pressure_value = float(values[1])
                    pressure_val1= calculate_P(sensor1_pressure_value,3+1/3,667,0)
                    pressure_val2= calculate_P(sensor2_pressure_value,3+1/3*1000,667,0)

                    sensor3_temp_value=float(values[2])
                    sensor4_temp_value=float(values[3])

                    temp_val_1=calculate_T(sensor3_temp_value)
                    temp_val_2=calculate_T(sensor4_temp_value)




                    #pressure data
                    
                    pub.publish(sensor1_pressure_value)
                    pub2.publish(sensor2_pressure_value)
                    pub3.publish(pressure_val1)
                    pub4.publish(pressure_val2)
                    #temp data
                    pub5.publish(sensor3_temp_value)
                    pub6.publish(sensor4_temp_value)
                    pub7.publish(temp_val_1)
                    pub8.publish(temp_val_2)


                    
        except UnicodeDecodeError:
            rospy.logwarn("Serial decode error")
        except ValueError:
            rospy.logwarn("Invalid float conversion")
        except serial.SerialException:
            rospy.logerr("Serial port error")
            break
        except rospy.ROSInterruptException:
            break
        time.sleep(0.001)  # maybe remove
        #if sleep is removed cpu stress is increased
        #need to check aliasing between this and esp32
        #this wait needs to be shorter than esp32
        




if __name__ == '__main__':
    try:
        read_serial()
    except:
        print('didnt work')