from setuptools import setup

setup(
    name='sensor_system',  # Replace with your package name
    version='0.0.0',
    packages=['sensor_system'],  # This should match your Python module's directory
    install_requires=[
        'rospy',
        'pyqt5',
        'pyqtgraph',
        'std_msgs',
        'serial'
    ],
)


