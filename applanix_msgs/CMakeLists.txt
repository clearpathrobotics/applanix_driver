cmake_minimum_required(VERSION 2.8.3)
project(applanix_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  rospy
)

# this is needed because this module includes a python library for inspecting
# the message mappings to Applanix.
catkin_python_setup()

SET(mapping_py "${CMAKE_CURRENT_SOURCE_DIR}/src/applanix_msgs/mapping.py")

# get the list of all of the messages for which to generate services
execute_process(COMMAND ${mapping_py} OUTPUT_VARIABLE gen_msg_names
  OUTPUT_STRIP_TRAILING_WHITESPACE)

MESSAGE("Generating AllMsgs.msg")
SET(all_msgs_path "${CMAKE_CURRENT_SOURCE_DIR}/msg/AllMsgs.msg")
# need to do this if the file doesn't exist already
execute_process(COMMAND ${mapping_py} -a OUTPUT_FILE ${all_msgs_path})

# custom command to generate AllMsgs
add_custom_command(
  OUTPUT ${all_msgs_path} COMMAND ${mapping_py} -a > ${all_msgs_path}
  DEPENDS ${mapping_py}
  )

add_custom_target(AllMsgs.msg DEPENDS ${mapping_py})

# go through each of the message names and generate a service
FOREACH(msg_name ${gen_msg_names})
  MESSAGE("Generating service definition for ${msg_name}")
  SET(srv_path "${CMAKE_CURRENT_SOURCE_DIR}/srv/Set${msg_name}.srv")
  execute_process(COMMAND ${mapping_py} -g ${msg_name} OUTPUT_FILE ${srv_path})
  SET(generated_srvs ${generated_srvs} "Set${msg_name}.srv;")

  add_custom_command(
    OUTPUT ${srv_path} 
    COMMAND ${mapping_py} -g ${msg_name} > ${srv_path}
    DEPENDS ${mapping_py}
    )

  add_custom_target("Set${msg_name}.srv" DEPENDS ${mapping_py})

ENDFOREACH(msg_name)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  AllMsgs.msg
  Ack.msg
  AidingSensorIntegrationControl.msg
  AidingSensorParams.msg
  BaseGNSSModemStatus.msg
  BaseGNSSSetup.msg
  BinaryMessageSelect.msg
  CalibratedInstallationParameters.msg
  CommonFooter.msg
  CommonHeader.msg
  COMPortMessages.msg
  COMPortParams.msg
  COMPortSetup.msg
  DGPSSourceControl.msg
  DMIData.msg
  Event.msg
  EventSetup.msg
  GAMSCalibrationControl.msg
  GAMS.msg
  GAMSParams.msg
  GeneralParams.msg
  GeneralStatus.msg
  GNSSAuxStatus.msg
  GNSSChannelStatus.msg
  GNSSControl.msg
  GNSSDGPSChannelStatus.msg
  GNSSDGPSStationDatabase.msg
  GNSSDGPSStation.msg
  GNSSDGPSStatus.msg
  GNSSSetup.msg
  GNSSStatus.msg
  Groups.msg
  IINSolutionStatus.msg
  IMUData.msg
  InstallationCalibrationControl.msg
  IntegrationDiagnosticsControl.msg
  IPAddress.msg
  LoggingControl.msg
  LoggingStatus.msg
  NavigationPerformance.msg
  NavigationSolution.msg
  NavModeControl.msg
  NMEAMessageSelect.msg
  OutputGroup.msg
  PortControl.msg
  PPSStatus.msg
  PreciseGravitySpecs.msg
  ProgramControl.msg
  RawData.msg
  RawDMI.msg
  RawPPS.msg
  SaveRestoreControl.msg
  TimeDistance.msg
  TimeSyncControl.msg
  UserAccuracySpecs.msg
  UserTimeStatus.msg
  Version.msg
)

## Service definitions generated above
add_service_files(
  FILES
  ${generated_srvs}
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package()
