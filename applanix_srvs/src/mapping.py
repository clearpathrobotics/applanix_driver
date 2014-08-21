#! /usr/bin/env python
# -*- coding: utf-8 -*-
#     _____
#    /  _  \
#   / _/ \  \
#  / / \_/   \
# /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
# \  / \_/ \  / /  _\| |  | __| / _ \ | ┌┐ \ | ┌┐ \ / _ \ |_   _|| | | |
#  \ \_/ \_/ /  | |  | |  | └─┐| |_| || └┘ / | └┘_/| |_| |  | |  | └─┘ |
#   \  \_/  /   | |_ | |_ | ┌─┘|  _  || |\ \ | |   |  _  |  | |  | ┌─┐ |
#    \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
#            ROBOTICS™
#
#
#  Copyright © 2012 Clearpath Robotics, Inc. 
#  All Rights Reserved
#  
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Clearpath Robotics, Inc. nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Please send comments, questions, or patches to skynet@clearpathrobotics.com
#
import roslib; roslib.load_manifest('applanix_msgs')
from applanix_msgs.msg import *


# applanix group number : ( topic name, ROS message class, [opts] )
groups = {
      1: ("nav", NavigationSolution),
      2: ("status/perf", NavigationPerformance),
      3: ("status/gnss/primary", GNSSStatus),
      4: ("imu", IMUData),
      5: ("events/1", Event),
      6: ("events/2", Event),
      7: ("status/pps", PPSStatus),
      8: ("status/logging", LoggingStatus),
      9: ("gams", GAMS),
      10: ("status/general", GeneralStatus),
      11: ("status/gnss/secondary", GNSSStatus),
      12: ("status/gnss/aux_1", GNSSAuxStatus),
      13: ("status/gnss/aux_2", GNSSAuxStatus),
      14: ("status/installation", CalibratedInstallationParameters),
      15: ("dmi", DMIData),
      17: ("status/user_time", UserTimeStatus),
      20: ("status/iin", IINSolutionStatus),
      21: ("status/base_gnss/1/modem", BaseGNSSModemStatus),
      22: ("status/base_gnss/2/modem", BaseGNSSModemStatus),
      23: ("raw/gnss/aux_1_display", RawData),
      24: ("raw/gnss/aux_2_display", RawData),
      25: ("status/dgps", GNSSDGPSStatus),
      26: ("status/dgps_stations", GNSSDGPSStationDatabase),
      30: ("events/3", Event),
      31: ("events/4", Event),
      32: ("events/5", Event),
      33: ("events/6", Event),
      99: ("status/version", Version),
      10001: ("raw/gnss/primary", RawData),
      10002: ("raw/imu", RawData),
      10003: ("raw/pps", RawPPS),
      10004: ("events/1", Event),
      10005: ("events/2", Event),
      10006: ("raw/dmi", RawDMI),
      10007: ("raw/gnss/aux_1", RawData),
      10008: ("raw/gnss/aux_2", RawData),
      10009: ("raw/gnss/secondary", RawData),
      10011: ("raw/base_gnss/1", RawData),
      10012: ("raw/base_gnss/2", RawData),
      }

msgs = {
      0: ("ack", Ack),
      20: ("general", GeneralParams),
      21: ("gams", GAMSParams),
      22: ("aiding_sensors", AidingSensorParams),
      24: ("user_accuracy", UserAccuracySpecs),
      30: ("primary_gnss_setup", GNSSSetup),
      31: ("secondary_gnss_setup", GNSSSetup),
      32: ("ip_address", IPAddress),
      33: ("event_setup", EventSetup),
      34: ("com_port_setup", COMPortSetup),
      35: ("nmea_message_select", NMEAMessageSelect),
      36: ("binary_message_select", BinaryMessageSelect),
      37: ("base_gnss_1_setup", BaseGNSSSetup),
      38: ("base_gnss_2_setup", BaseGNSSSetup),
      40: ("precise_gravity", PreciseGravitySpecs),
      41: ("primary_dgps_source", DGPSSourceControl),
      50: ("nav_mode", NavModeControl),
      51: ("display_port", PortControl),
      52: ("primary_data_port", PortControl),
      53: ("logging_port", LoggingControl),
      54: ("save_restore", SaveRestoreControl),
      55: ("time_sync", TimeSyncControl),
      57: ("installation_calibration", InstallationCalibrationControl),
      58: ("gams_calibration", GAMSCalibrationControl),
      61: ("secondary_data_port", PortControl),
      90: ("program", ProgramControl),
      91: ("gnss", GNSSControl),
      92: ("integration_diagnostics", IntegrationDiagnosticsControl),
      93: ("aiding_sensor_integration", AidingSensorIntegrationControl),
    }


# Message excluded from the generated AllMsgs aggregate message.
all_msgs_exclude = set([Ack, SaveRestoreControl, InstallationCalibrationControl, 
    GAMSCalibrationControl, ProgramControl, GNSSControl])
for name, msg in msgs.values():
  if msg in all_msgs_exclude:
    msg.in_all_msgs = False
  else:
    msg.in_all_msgs = True


if __name__ == '__main__':
  from pprint import pprint
  pprint((groups, msgs))
