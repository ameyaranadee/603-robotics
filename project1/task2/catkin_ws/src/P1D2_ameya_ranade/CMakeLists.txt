cmake_minimum_required(VERSION 3.0.2)
project(P1D2_ameya_ranade)

find_package(catkin REQUIRED COMPONENTS
    rospy
    geometry_msgs
    turtlesim
    std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
    scripts/P1D2_ameya_ranade.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
