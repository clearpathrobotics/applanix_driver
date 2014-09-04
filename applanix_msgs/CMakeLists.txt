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

## Generate messages in the 'msg' folder
add_message_files(
  FILES
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

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package()
