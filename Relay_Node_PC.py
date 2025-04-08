#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPalette, QColor

import rospy
from std_msgs.msg import Bool, Float32, String

class ControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Control Panel")
        self.setMinimumSize(800, 600)
        self.main_widget = QWidget()
        self.setCentralWidget(self.main_widget)
        self.main_layout = QVBoxLayout(self.main_widget)

        self.relay_states = [False]*4
        self.emergency_stop = False
        self.servo_angle = 0

        self.init_ros()
        self.create_ui()
        self.apply_styles()
        self.statusBar().showMessage("ROS Publishing Mode")

    def init_ros(self):
        rospy.init_node("control_gui_node", anonymous=True)
        self.relay_pub = rospy.Publisher("/relay_states", String, queue_size=10)
        self.estop_pub = rospy.Publisher("/emergency_stop", Bool, queue_size=10)
        self.servo_pub = rospy.Publisher("/servo_angle", Float32, queue_size=10)

    def create_ui(self):
        self.create_relay_section()
        self.create_emergency_section()
        self.create_servo_section()
        self.main_layout.addStretch()

    def create_relay_section(self):
        group = QGroupBox("Relays")
        layout = QGridLayout()

        self.relay_buttons = []
        for i in range(4):
            button = QPushButton(f"Relay {i+1}: OFF")
            button.setCheckable(True)
            button.toggled.connect(lambda checked, idx=i: self.toggle_relay(idx, checked))
            self.relay_buttons.append(button)
            layout.addWidget(QLabel(f"Relay {i+1}:"), i, 0)
            layout.addWidget(button, i, 1)

        group.setLayout(layout)
        self.main_layout.addWidget(group)

    def toggle_relay(self, index, state):
        self.relay_states[index] = state
        self.relay_buttons[index].setText(f"Relay {index+1}: {'ON' if state else 'OFF'}")
        relay_str = ','.join(['1' if s else '0' for s in self.relay_states])
        self.relay_pub.publish(f"{relay_str}")

    def create_emergency_section(self):
        group = QGroupBox("Emergency Stop")
        layout = QVBoxLayout()

        self.estop_button = QPushButton("EMERGENCY STOP")
        self.estop_button.setStyleSheet("background-color: #d9534f; color: white; font-weight: bold; font-size: 16px;")
        self.estop_button.clicked.connect(self.activate_emergency_stop)
        layout.addWidget(self.estop_button)

        self.estop_status = QLabel("Kill Switch: Ready")
        layout.addWidget(self.estop_status)

        group.setLayout(layout)
        self.main_layout.addWidget(group)

    def activate_emergency_stop(self):
        self.emergency_stop = True
        self.estop_pub.publish(True)
        self.estop_status.setText("Kill Switch: ACTIVATED")

    def create_servo_section(self):
        group = QGroupBox("Servo")
        layout = QVBoxLayout()

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 270)
        # Update the label continuously but do not publish on each change.
        self.slider.valueChanged.connect(self.update_servo_label)
        # Publish only when the slider is released.
        self.slider.sliderReleased.connect(self.publish_servo)
        layout.addWidget(QLabel("Servo Angle (0°–270°)"))
        layout.addWidget(self.slider)

        self.servo_label = QLabel("Current Position: 0°")
        self.servo_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.servo_label)

        group.setLayout(layout)
        self.main_layout.addWidget(group)

    def update_servo_label(self, value):
        self.servo_angle = value
        self.servo_label.setText(f"Current Position: {value}°")

    def publish_servo(self):
        self.servo_pub.publish(self.servo_angle)

    def apply_styles(self):
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor("#2b2b2b"))
        palette.setColor(QPalette.WindowText, QColor("#ffffff"))
        palette.setColor(QPalette.Base, QColor("#3c3c3c"))
        palette.setColor(QPalette.AlternateBase, QColor("#4e4e4e"))
        palette.setColor(QPalette.ToolTipBase, QColor("#ffffff"))
        palette.setColor(QPalette.ToolTipText, QColor("#ffffff"))
        palette.setColor(QPalette.Text, QColor("#ffffff"))
        palette.setColor(QPalette.Button, QColor("#444444"))
        palette.setColor(QPalette.ButtonText, QColor("#ffffff"))
        palette.setColor(QPalette.BrightText, QColor("#ff0000"))
        palette.setColor(QPalette.Highlight, QColor("#666666"))
        palette.setColor(QPalette.HighlightedText, QColor("#ffffff"))
        self.setPalette(palette)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = ControlGUI()
    gui.show()
    sys.exit(app.exec_())
