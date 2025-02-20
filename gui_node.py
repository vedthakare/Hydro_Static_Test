import sys
import os
import rospy
from std_msgs.msg import Int32
from PyQt6 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

class RosSubscriberThread(QtCore.QThread):
    data_received = QtCore.pyqtSignal(float)

    def __init__(self, topic_name="remote_sensor_data", parent=None):
        super().__init__(parent)
        self.topic_name = topic_name

    def run(self):
        rospy.init_node('gui_node', anonymous=True)
        rospy.Subscriber(self.topic_name, Int32, self.ros_callback)
        rospy.spin()

    def ros_callback(self, msg):
        self.data_received.emit(float(msg.data))

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Sensor Data Display")
        self.resize(800, 600)
        self.data = []
        self.plot_data = []
        self.setup_ui()
        
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)
        
        self.ros_thread = RosSubscriberThread()
        self.ros_thread.data_received.connect(self.handle_new_data)
        self.ros_thread.start()

    def setup_ui(self):
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QVBoxLayout(central_widget)

        # Real-time plot
        self.plot_widget = pg.PlotWidget(title="Sensor Data")
        self.plot_widget.setBackground("#2d2d2d")
        main_layout.addWidget(self.plot_widget)
        self.plot_curve = self.plot_widget.plot(pen=pg.mkPen(color=(42, 130, 218), width=2))

        # Status label
        self.status_label = QtWidgets.QLabel("Waiting for data...")
        main_layout.addWidget(self.status_label)

    def handle_new_data(self, value):
        self.data.append(value)
        self.plot_data.append(value)
        self.status_label.setText(f"Latest Value: {value:.2f}")

    def update_plot(self):
        self.plot_curve.setData(self.plot_data[-1000:])  # Show last 1000 points

    def closeEvent(self, event):
        self.ros_thread.quit()
        self.ros_thread.wait()
        event.accept()

def set_modern_style(app):
    app.setStyle("Fusion")
    palette = QtGui.QPalette()
    palette.setColor(QtGui.QPalette.ColorRole.Window, QtGui.QColor(53, 53, 53))
    palette.setColor(QtGui.QPalette.ColorRole.WindowText, QtCore.Qt.GlobalColor.white)
    palette.setColor(QtGui.QPalette.ColorRole.Base, QtGui.QColor(35, 35, 35))
    palette.setColor(QtGui.QPalette.ColorRole.AlternateBase, QtGui.QColor(53, 53, 53))
    palette.setColor(QtGui.QPalette.ColorRole.ToolTipBase, QtCore.Qt.GlobalColor.white)
    palette.setColor(QtGui.QPalette.ColorRole.ToolTipText, QtCore.Qt.GlobalColor.white)
    palette.setColor(QtGui.QPalette.ColorRole.Text, QtCore.Qt.GlobalColor.white)
    palette.setColor(QtGui.QPalette.ColorRole.Button, QtGui.QColor(53, 53, 53))
    palette.setColor(QtGui.QPalette.ColorRole.ButtonText, QtCore.Qt.GlobalColor.white)
    palette.setColor(QtGui.QPalette.ColorRole.BrightText, QtCore.Qt.GlobalColor.red)
    palette.setColor(QtGui.QPalette.ColorRole.Link, QtGui.QColor(42, 130, 218))
    palette.setColor(QtGui.QPalette.ColorRole.Highlight, QtGui.QColor(42, 130, 218))
    palette.setColor(QtGui.QPalette.ColorRole.HighlightedText, QtCore.Qt.GlobalColor.black)
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
    app = QtWidgets.QApplication(sys.argv)
    set_modern_style(app)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

