#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

# Callback for PT1 voltage conversion
def convert_pt1_v(msg):
    voltage = msg.data
    actual = pressure_conversion(voltage)
    pt1_pub.publish(actual)  # Publish converted pressure to pt1_value

# Callback for PT2 voltage conversion
def convert_pt2_v(msg):
    voltage = msg.data
    actual = pressure_conversion(voltage)
    pt2_pub.publish(actual)

# Callback for TC1 voltage conversion
def convert_tc1_v(msg):
    voltage = msg.data
    actual = thermocouple_conversion(voltage)
    tc1_pub.publish(actual)

# Callback for TC2 voltage conversion
def convert_tc2_v(msg):
    voltage = msg.data
    actual = thermocouple_conversion(voltage)
    tc2_pub.publish(actual)

# Callback for load cell voltage conversion
def convert_loadcell(msg):
    voltage = msg.data
    actual = loadcell_conversion(voltage)
    loadcell_pub.publish(actual)

# Pressure sensor conversion function
def pressure_conversion(voltage):
    """
    Convert voltage to pressure in PSI using sensor-specific formula:
    P = ((V_out - 0.1 * V_supply) * (P_max - P_min)) / (0.8 * V_supply)
    """
    V_out = voltage
    V_supply = (3 + 1/3) * 1000  # mV, assuming 3.33 V supply
    P_max = 667  # max pressure in PSI
    P_min = 0    # min pressure

    P = ((V_out - 0.1 * V_supply) * (P_max - P_min)) / (0.8 * V_supply)
    return P

# Thermocouple voltage to temperature conversion
def thermocouple_conversion(voltage):
    """
    Convert thermocouple voltage (V_o) to temperature (Celsius)
    Formula derived from a linear approximation.
    """
    V_o = voltage
    T = (V_o - 1.25) / (5e-3)  # assuming 5 mV/°C sensitivity
    return T

# Load cell conversion from digital value to force (lbs)
def loadcell_conversion(Val):
    """
    Convert raw load cell value to weight in pounds-force (lbf)
    using a calibration factor from a known weight.
    """
    known_weight = 2  # lbs, known calibration weight
    cal_val_ = 5      # raw value recorded with known weight
    cal_factor = known_weight / cal_val_

    weight = Val * cal_factor
    return weight

if __name__ == '__main__':
    rospy.init_node('conversion_node')

    # Publishers for converted sensor values
    pt1_pub = rospy.Publisher('pt1_value', Float32, queue_size=10)
    pt2_pub = rospy.Publisher('pt2_value', Float32, queue_size=10)
    tc1_pub = rospy.Publisher('tc1_value', Float32, queue_size=10)
    tc2_pub = rospy.Publisher('tc2_value', Float32, queue_size=10)
    loadcell_pub = rospy.Publisher('loadcell_value', Float32, queue_size=10)

    # Subscribers for voltage readings from sensors
    rospy.Subscriber('pt1_v', Float32, convert_pt1_v)
    rospy.Subscriber('pt2_v', Float32, convert_pt2_v)
    rospy.Subscriber('tc1_v', Float32, convert_tc1_v)
    rospy.Subscriber('tc2_v', Float32, convert_tc2_v)
    rospy.Subscriber('loadcell', Float32, convert_loadcell)

    rospy.spin()
