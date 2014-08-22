cmake_minimum_required(VERSION 2.8.3)
project(applanix_bridge)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
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
