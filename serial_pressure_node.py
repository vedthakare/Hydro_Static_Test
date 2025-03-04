#!/usr/bin/env python3
import rospy
import serial
import time
import RPi.GPIO as GPIO
from std_msgs.msg import Float32

# GPIO Setup
ESP32_ENABLE_PIN = 17  # Use the correct GPIO pin
GPIO.setmode(GPIO.BCM)
GPIO.setup(ESP32_ENABLE_PIN, GPIO.OUT)
GPIO.output(ESP32_ENABLE_PIN, GPIO.LOW)  # Ensure it's low initially

# Serial Setup
ser = serial.Serial('/dev/ttyS0', 115200)

rospy.init_node('serial_node')
pub = rospy.Publisher('voltage_pressure_data', Float32, queue_size=10)
pub2 = rospy.Publisher('voltage_pressure_data_2', Float32, queue_size=10)

def read_serial():
    # Signal to ESP32 to start
    GPIO.output(ESP32_ENABLE_PIN, GPIO.HIGH)
    time.sleep(0.5)  # Allow time for ESP32 to start

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
        time.sleep(0.001)  # maybe remove

if __name__ == '__main__':
    try:
        read_serial()
    finally:
        GPIO.cleanup()  # Reset GPIO states on exit
