#!/usr/bin/env python

# ROS
import rospy
import applanix_msgs.msg

# Package modules
from data import DataPort
from control import ControlPort
from monitor import Monitor

# Standard
import socket
import struct
from cStringIO import StringIO
import time

import translator
from handlers import NullHandler, GroupHandler, MessageHandler, AckHandler

PORTS_DATA = {
    "realtime": 5602,
    "logging": 5603
    }
PORT_CONTROL = 5601

DEFAULT_IP = '192.168.53.100'
PREFIX_DEFAULTS = {
    "raw": True,
    "dmi": True,
    "status": True,
    "events": True
    }

SOCKET_TIMEOUT=10.0
socks = []
ports = {}
monitor = Monitor(ports)


def main():
  rospy.init_node('applanix_bridge')

  # Where to find the device. It does not support DHCP, so you'll need
  # to connect initially to the factory default IP in order to change it.
  ip = rospy.get_param('ip', DEFAULT_IP)

  # Select between realtime and logging data ports. The logging data is
  # optimized for completeness, whereas the realtime data is optimized for
  # freshness.
  data_port = PORTS_DATA[rospy.get_param('data', "realtime")]

  # Disable this to not connect to the control socket (for example, if you
  # want to control the device using the Applanix POS-LV software rather
  # than ROS-based services.
  control_enabled = rospy.get_param('control', True)

  # Disable any of these to hide auxiliary topics which you may not be
  # interested in.
  exclude_prefixes = []
  for prefix, default in PREFIX_DEFAULTS.items():
    if not rospy.get_param('include_%s' % prefix, default):
      exclude_prefixes.append(prefix)

  ports['data'] = DataPort(create_sock('data', ip, data_port),
                           exclude_prefixes=exclude_prefixes)
  if control_enabled:
    ports['control'] = ControlPort(create_sock('control', ip, PORT_CONTROL))

  for name, port in ports.items():
    port.start()
    rospy.loginfo("Port %s thread started." % name)
  monitor.start()

  rospy.on_shutdown(shutdown)
  rospy.spin()


def create_sock(name, ip, port):
  try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ip_port = (ip, port)
    #if name != "control": 
    sock.connect(ip_port)
    rospy.loginfo("Successfully connected to %%s port at %s:%d" % ip_port % name)
  except socket.error:
    rospy.logfatal("Couldn't connect to %%s port at %s:%d." % ip_port % name)
    raise
  socks.append(sock)
  return sock


def shutdown():
  monitor.finish.set()
  monitor.join()
  rospy.loginfo("Thread monitor finished.") 
  for name, port in ports.items():
    port.finish.set()
    port.join()
    rospy.loginfo("Port %s thread finished." % name) 
  for sock in socks:
    sock.shutdown(socket.SHUT_RDWR)
    sock.close()
  rospy.loginfo("Sockets closed.") 
