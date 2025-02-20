import rospy
from std_msgs.msg import Int32
import tkinter as tk

def update_label(msg):
    label.config(text=f"Sensor Data: {msg.data}")

def gui():
    global label
    root = tk.Tk()
    root.title("Sensor Data Display")
    label = tk.Label(root, text="Waiting for data...")
    label.pack()
    rospy.init_node('gui_node')
    rospy.Subscriber('sensor_data', Int32, update_label)
    root.mainloop()

if __name__ == '__main__':
    gui()

