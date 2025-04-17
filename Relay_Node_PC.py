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
        self.servo_angle = 100
        self.servo_angle_2 = 0

        self.init_ros()
        self.create_ui()
        self.apply_styles()
        self.statusBar().showMessage("ROS Publishing Mode")

    def init_ros(self):
        rospy.init_node("control_gui_node", anonymous=True)
        self.relay_pub = rospy.Publisher("/relay_states", String, queue_size=10)
        self.estop_pub = rospy.Publisher("/emergency_stop", Bool, queue_size=10)
        self.servo_pub = rospy.Publisher("/servo_angle", Float32, queue_size=3)
        self.servo2_pub= rospy.Publisher("/servo_angle_2", Float32, queue_size=3)

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
        relay_str = ','.join(['0' if s else '1' for s in self.relay_states])
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
        group = QGroupBox("Servo Controls")
        main_layout = QVBoxLayout()
        
        # Create a horizontal layout for the two servo controls
        servo_layout = QHBoxLayout()
        
        # First Servo Control
        first_servo_layout = QVBoxLayout()
        first_servo_layout.addWidget(QLabel("Servo 1 Angle (20°–100°)"))
        
        # Add a horizontal layout for slider and text input
        slider1_layout = QHBoxLayout()
        
        # Create slider for first servo
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(20, 100)
        self.slider.valueChanged.connect(self.update_servo)
        slider1_layout.addWidget(self.slider, 4)  # Give slider more space
        
        # Create text input for first servo
        self.servo_input = QLineEdit()
        self.servo_input.setMaximumWidth(60)
        self.servo_input.setText("100")
        self.servo_input.returnPressed.connect(self.update_servo_from_text)
        slider1_layout.addWidget(self.servo_input, 1)  # Give input box less space
        
        first_servo_layout.addLayout(slider1_layout)
        
        self.servo_label = QLabel("Current Position: 100°")
        self.servo_label.setAlignment(Qt.AlignCenter)
        first_servo_layout.addWidget(self.servo_label)
        
        # Add first servo layout to the horizontal layout
        servo_layout.addLayout(first_servo_layout)
        
        # Second Servo Control
        second_servo_layout = QVBoxLayout()
        second_servo_layout.addWidget(QLabel("Servo 2 Angle (15°–95°)"))
        
        # Add a horizontal layout for slider and text input
        slider2_layout = QHBoxLayout()
        
        # Create slider for second servo
        self.slider2 = QSlider(Qt.Horizontal)
        self.slider2.setRange(15, 95)
        self.slider2.valueChanged.connect(self.update_servo2)
        slider2_layout.addWidget(self.slider2, 4)  # Give slider more space
        
        # Create text input for second servo
        self.servo2_input = QLineEdit()
        self.servo2_input.setMaximumWidth(60)
        self.servo2_input.setText("95")
        self.servo2_input.returnPressed.connect(self.update_servo2_from_text)
        slider2_layout.addWidget(self.servo2_input, 1)  # Give input box less space
        
        second_servo_layout.addLayout(slider2_layout)
        
        self.servo2_label = QLabel("Current Position: 95°")
        self.servo2_label.setAlignment(Qt.AlignCenter)
        second_servo_layout.addWidget(self.servo2_label)
        
        # Add second servo layout to the horizontal layout
        servo_layout.addLayout(second_servo_layout)
        
        # Add the horizontal layout to the main layout
        main_layout.addLayout(servo_layout)
        
        group.setLayout(main_layout)
        self.main_layout.addWidget(group)

    def update_servo(self, value):
        self.servo_angle = value
        self.servo_label.setText(f"Current Position: {value}°")
        self.servo_input.setText(str(value))
        self.servo_pub.publish(Float32(value))

    def update_servo_from_text(self):
        try:
            value = int(self.servo_input.text())
            # Ensure value is within valid range
            value = max(20, min(100, value))
            self.slider.setValue(value)
            # The slider's valueChanged signal will trigger update_servo
        except ValueError:
            # Reset to current value if input is invalid
            self.servo_input.setText(str(self.servo_angle))

    def update_servo2(self, value):
        self.servo_angle_2 = value
        self.servo2_label.setText(f"Current Position: {value}°")
        self.servo2_input.setText(str(value))
        self.servo2_pub.publish(Float32(value))

    def update_servo2_from_text(self):
        try:
            value = int(self.servo2_input.text())
            # Ensure value is within valid range
            value = max(15, min(95, value))
            self.slider2.setValue(value)
            # The slider's valueChanged signal will trigger update_servo2
        except ValueError:
            # Reset to current value if input is invalid
            self.servo2_input.setText(str(self.servo_angle_2))

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