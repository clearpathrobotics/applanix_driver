cmake_minimum_required(VERSION 2.8.3)
project(applanix_bridge)

find_package(catkin REQUIRED COMPONENTS
  applanix_msgs
  rospy
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES applanix_bridge
#  CATKIN_DEPENDS common_msgs diagnostic_msgs geometry orocos_kinematics_dynamics rospy
#  DEPENDS python-pcapy python-impacket
)

install(PROGRAMS
  bin/bridge
  bin/diagnostics_publisher
  bin/params
  bin/publisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# add launch scripts to install
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
