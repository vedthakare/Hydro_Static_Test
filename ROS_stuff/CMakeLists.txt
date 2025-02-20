cmake_minimum_required(VERSION 3.0.2)
project(sensor_system)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()  # Ensure Python scripts are handled correctly

catkin_package()

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# If you have specific Python scripts that should be executable
install(PROGRAMS
  scripts/my_script.py  # Add more scripts here if needed
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


