cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Generate a corresponding service for each message in applanix_msg
# which defines an Applanix $MSG packet, as well as an aggregate
# message called AllMsgs.msg.
execute_process(COMMAND rosrun applanix_generated_msgs generate.py
  OUTPUT_VARIABLE generated_srvs)

rosbuild_add_generated_srvs(${generated_srvs})
rosbuild_add_generated_msgs(AllMsgs.msg)

rosbuild_init()
rosbuild_genmsg()
rosbuild_gensrv()
