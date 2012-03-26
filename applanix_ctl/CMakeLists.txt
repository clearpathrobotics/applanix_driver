cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

execute_process(COMMAND rosrun applanix_ctl generate.py
  OUTPUT_VARIABLE generated_srvs)

MESSAGE("${generated_srvs}")
rosbuild_add_generated_srvs(${generated_srvs})
rosbuild_add_generated_msgs(AllMsgs.msg)


rosbuild_init()

rosbuild_genmsg()
rosbuild_gensrv()
