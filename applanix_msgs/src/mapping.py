#!/usr/bin/python
import roslib; roslib.load_manifest('applanix_msgs')
from applanix_msgs.msg import *

# applanix group number : ( topic name, ROS message class )
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
      25: None,
      26: None,
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
      20: ("general", GeneralParams),
      21: ("gams", GAMSParams),
      22: ("aiding_sensors", AidingSensorParams),
    }

ack = {
      0: ("ack", Ack),
    }

if __name__ == '__main__':
  from pprint import pprint
  pprint((groups, msgs, ack))
