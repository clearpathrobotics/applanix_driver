#!/usr/bin/env python

# ROS
import rospy

# Package modules
from data import DataPort
from control import ControlPort

# Standard
from pprint import pprint
import socket


PORTS_DATA = {
    "realtime": 5602,
    "logging": 5603
    }
PORT_CONTROL = 5601

DEFAULT_IP = '10.25.0.51'

PREFIX_DEFAULTS = {
    "raw": True,
    "dmi": True,
    "status": True,
    "events": True
    }

SOCKET_TIMEOUT=10.0


def main():
  rospy.init_node('applanix_bridge')

  # Where to find the device. It does not support DHCP, so you'll need
  # to connect initially to the factory default IP in order to change it.
  ip = rospy.get_param('~ip', DEFAULT_IP)

  # Select between realtime and logging data ports. The logging data is
  # optimized for completeness, whereas the realtime data is optimized for
  # freshness.
  data_port = rospy.get_param('~data', "realtime")

  # Disable this to not connect to the control socket (for example, if you
  # want to control the device using the Applanix POS-LV software rather
  # than ROS-based services.
  control_enabled = rospy.get_param('~control', True)

  # Disable any of these to hide auxiliary topics which you may not be
  # interested in. eg, _include_raw:=false
  exclude_prefixes = []
  for prefix, default in PREFIX_DEFAULTS.items():
    if not rospy.get_param('~include_%s' % prefix, default):
      exclude_prefixes.append(prefix)

  socks = {}
  ports = []

  # Could support the Display port here without too much effort.

  try:
    socks['data'] = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    socks['data'].settimeout(SOCKET_TIMEOUT)
    ip_port = (ip, PORTS_DATA[data_port])
    socks['data'].connect(ip_port)
    rospy.loginfo("Successfully connected to data port at %s:%d" % ip_port)
  except socket.error:
    rospy.logfatal("Couldn't connect to data port at %s:%d." % ip_port)
    exit(1)
  ports.append(DataPort(socks['data'], exclude_prefixes=exclude_prefixes))

  control_sock = None
  if control_enabled:
    try:
      socks['control'] = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      socks['control'].settimeout(SOCKET_TIMEOUT)
      ip_port = (ip, PORT_CONTROL)
      socks['control'].connect(ip_port)
      rospy.loginfo("Successfully connected to control port at %s:%d" % ip_port)
    except socket.error:
      rospy.logfatal("Couldn't connect to control port at %s:%d." % ip_port)
      exit(1)
    ports.append(ControlPort(socks['control']))

  for port in ports:
    port.start()
    rospy.loginfo("%s thread started." % port.__class__.__name__)

  # Main node operation takes place in DataPort and ControlPort.
  while not rospy.is_shutdown():
    rospy.sleep(1.0)
    for port in ports:
      if not port.is_alive():
        rospy.logerr("%s thread died. Signalling node shutdown." % port.__class__.__name__)
        rospy.signal_shutdown("Node lost thread.")

  rospy.loginfo("Beginning shutdown.")
  
  for port in ports:
    port.finish.set()
    port.join()
    rospy.loginfo("%s thread finished." % port.__class__.__name__)

  for name, sock in socks.items():
    rospy.loginfo("Closing %s socket" % name)
    sock.close()
  
  rospy.loginfo("Shutdown complete.")
