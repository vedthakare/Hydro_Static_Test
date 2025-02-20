#!/usr/bin/env python3
import sys
import os
import rospy
from std_msgs.msg import Int32
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

class RosSubscriberThread(QtCore.QThread):
    data_received = QtCore.pyqtSignal(str, float)  # topic_name, value

    def __init__(self, topic_names=None, parent=None):
        super().__init__(parent)
        self.topic_names = topic_names or ["remote_sensor_data"]

    def run(self):
        for topic in self.topic_names:
            rospy.Subscriber(topic, Int32, lambda msg, topic=topic: self.ros_callback(topic, msg))
        rospy.spin()

    def ros_callback(self, topic_name, msg):
        self.data_received.emit(topic_name, float(msg.data))

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, topic_names=None):
        super().__init__()
        self.topic_names = topic_names or ["remote_sensor_data"]
        self.setWindowTitle("Sensor Data Display")
        self.resize(800, 600)
        
        # Dictionary to store data for each topic
        self.data_streams = {topic: [] for topic in self.topic_names}
        self.setup_ui()
        
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)
        
        self.ros_thread = RosSubscriberThread(self.topic_names)
        self.ros_thread.data_received.connect(self.handle_new_data)
        self.ros_thread.start()

    def setup_ui(self):
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QVBoxLayout(central_widget)

        # Real-time plot
        self.plot_widget = pg.PlotWidget(title="Sensor Data")
        self.plot_widget.setBackground("#2d2d2d")
        self.plot_widget.addLegend()
        main_layout.addWidget(self.plot_widget)
        
        # Generate distinct colors for each topic
        colors = self.generate_colors(len(self.topic_names))
        
        # Create plot curves and status labels for each topic
        self.plot_curves = {}
        self.status_labels = {}
        
        status_layout = QtWidgets.QGridLayout()
        for idx, topic in enumerate(self.topic_names):
            # Create plot curve
            self.plot_curves[topic] = self.plot_widget.plot(
                pen=pg.mkPen(color=colors[idx], width=2),
                name=f"Sensor {idx + 1} ({topic})"
            )
            
            # Create status label
            self.status_labels[topic] = QtWidgets.QLabel(f"{topic}: Waiting for data...")
            status_layout.addWidget(self.status_labels[topic], idx // 2, idx % 2)

        main_layout.addLayout(status_layout)

    def generate_colors(self, n):
        # Generate visually distinct colors
        colors = []
        for i in range(n):
            hue = (i * 137.508) % 360  # Use golden angle to generate distinct hues
            color = QtGui.QColor.fromHsv(int(hue), 200, 255)
            colors.append(color.getRgb()[:3])
        return colors

    def handle_new_data(self, topic, value):
        self.data_streams[topic].append(value)
        self.status_labels[topic].setText(f"{topic}: {value:.2f}")

    def update_plot(self):
        for topic, curve in self.plot_curves.items():
            curve.setData(self.data_streams[topic][-1000:])  # Show last 1000 points

    def closeEvent(self, event):
        self.ros_thread.quit()
        self.ros_thread.wait()
        event.accept()

def set_modern_style(app):
    app.setStyle("Fusion")
    palette = QtGui.QPalette()
    palette.setColor(QtGui.QPalette.Window, QtGui.QColor(53, 53, 53))
    palette.setColor(QtGui.QPalette.WindowText, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.Base, QtGui.QColor(35, 35, 35))
    palette.setColor(QtGui.QPalette.AlternateBase, QtGui.QColor(53, 53, 53))
    palette.setColor(QtGui.QPalette.ToolTipBase, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.ToolTipText, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.Text, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.Button, QtGui.QColor(53, 53, 53))
    palette.setColor(QtGui.QPalette.ButtonText, QtCore.Qt.white)
    palette.setColor(QtGui.QPalette.BrightText, QtCore.Qt.red)
    palette.setColor(QtGui.QPalette.Link, QtGui.QColor(42, 130, 218))
    palette.setColor(QtGui.QPalette.Highlight, QtGui.QColor(42, 130, 218))
    palette.setColor(QtGui.QPalette.HighlightedText, QtCore.Qt.black)
    app.setPalette(palette)

    qss = """
    QMainWindow {
        background-color: #353535;
    }
    QPushButton {
        background-color: #3c3c3c;
        color: white;
        border: none;
        border-radius: 5px;
        padding: 8px 16px;
    }
    QPushButton:hover {
        background-color: #505050;
    }
    QPushButton:pressed {
        background-color: #3c3c3c;
    }
    QLabel {
        color: white;
    }
    """
    app.setStyleSheet(qss)

if __name__ == '__main__':
    rospy.init_node('gui_node', anonymous=True)
    app = QtWidgets.QApplication(sys.argv)
    set_modern_style(app)
    
    # Define the topics you want to subscribe to
    topics = [
        "remote_sensor_data",
        "remote_sensor_data_2",
        # Add more topics here as needed
    ]
    
    window = MainWindow(topics)
    window.show()
    sys.exit(app.exec())

