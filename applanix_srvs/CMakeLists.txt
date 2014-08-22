cmake_minimum_required(VERSION 2.8.3)
project(applanix_srvs)

find_package(catkin REQUIRED COMPONENTS
  applanix_msgs
  message_generation
  rospy
)

# run the code generation script to produce all of the services and AllMsgs
execute_process(COMMAND rosrun applanix_srvs generate.py
  OUTPUT_VARIABLE generated_srvs)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  AllMsgs.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ${generated_srvs}
)

generate_messages(
  DEPENDENCIES
  applanix_msgs
)
