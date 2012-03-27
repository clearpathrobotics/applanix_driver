#!/usr/bin/env python

import roslib, rospy, roslib.genpy
import applanix_bridge.msg as common
from applanix_ctl.msg import AllMsgs
import applanix_ctl.srv as msg_srv
from mapping import groups, msgs

from pprint import pprint
import socket
import struct
from cStringIO import StringIO
import select
import time

import translator
from handlers import NullHandler, GroupHandler, MessageHandler, AckHandler

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
HEADER_LEN = 8
SELECT_TIMEOUT = 0.1
ALLMSGS_SEND_TIMEOUT = rospy.Duration.from_sec(0.1)

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
  # want to control the device using the Applanix POS-LV software.
  control = rospy.get_param('~control', True)

  # Disable any of these to hide auxiliary topics which you may not be
  # interested in.
  exclude_prefixes = []
  for prefix, default in PREFIX_DEFAULTS.items():
    if not rospy.get_param('~include_%s' % prefix, default):
      exclude_prefixes.append(prefix)

  try:
    data_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ip_port = (ip, PORTS_DATA[data_port])
    data_sock.connect(ip_port)
    rlist = [data_sock]
  except socket.error:
    rospy.logfatal("Couldn't connect to data port at %s:%d." % ip_port)
    exit(1)

  control_sock = None
  if control:
    try:
      control_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      ip_port = (ip, PORT_CONTROL)
      control_sock.connect(ip_port)
      rlist.append(control_sock)
    except socket.error:
      rospy.logfatal("Couldn't connect to control port at %s:%d." % ip_port)
      data_sock.close()
      exit(1)

  all_msgs = AllMsgs()
  all_msgs_pub = rospy.Publisher("config", AllMsgs, latch=True)

  handlers = {}
  null_handler = NullHandler()

  def mk_group_handler(group_tuple):
    topic, msg_cls = group_tuple
    if not topic:
      return null_handler
    for prefix in exclude_prefixes:
      if topic.startswith(prefix): 
        return null_handler
    return GroupHandler(topic, msg_cls)

  for group_num in groups.keys():
    handlers[(common.CommonHeader.START_GROUP, group_num)] = mk_group_handler(groups[group_num])

  def mk_msg_handler(msg_tuple):
    name, msg_cls = msg_tuple
    return MessageHandler(name, msg_cls, all_msgs)

  for msg_num in msgs.keys():
    handlers[(common.CommonHeader.START_MESSAGE, msg_num)] = mk_msg_handler(msgs[msg_num])


  counters = {}
  bad = set()
  header_translator = translator.get(common.CommonHeader)

  try:
    while not rospy.is_shutdown():
      try:
        rlist_ready, wlist_ready, xlist_ready = select.select(rlist, [], [], SELECT_TIMEOUT)
      except select.error:
        rospy.loginfo("Exiting on select break.")
        break

      ready = set(rlist_ready)

      header = common.CommonHeader()
      data = StringIO(data_sock.recv(HEADER_LEN))
      header_translator.deserialize(data, header)
      pkt = (str(header.start).encode('string_escape'), header.id)
      if pkt[0] == '$MSG':
        print pkt
        print all_msgs

      data = StringIO(data_sock.recv(header.length))

      try:
        #  print [ord(a) for a in data.getvalue()[26:100]]
        handlers[pkt].handle(data)

      except translator.TranslatorError:
        if pkt not in bad:
          rospy.logwarn("Error parsing %s.%d" % pkt)
          bad.add(pkt)

      except KeyError:
        warn = True
        if pkt in counters:
          # Already seen this message-- only warn once.
          warn = False
        if pkt[0] in ('$GRP', '$MSG') and pkt[1] >= 20000:
          # Internal Applanix diagnostic messages.
          warn = False
        if warn:
          rospy.logwarn("Unimplemented %s.%d" % pkt)

      if pkt not in counters:
        counters[pkt] = 0
      counters[pkt] += 1

      if all_msgs.last_changed > all_msgs.last_sent and \
          rospy.get_rostime() > all_msgs.last_changed + ALLMSGS_SEND_TIMEOUT:
        all_msgs_pub.publish(all_msgs)
        print "PUBLISH!!!"
        all_msgs.last_sent = rospy.get_rostime()


  finally:
    if control_sock:
      print "Closing control socket"
      #control_sock.shutdown(socket.SHUT_RDWR)
      #time.sleep(0.3)
      control_sock.close()

    print "Closing data socket"
    #data_sock.shutdown(socket.SHUT_RDWR)
    #time.sleep(0.3)
    data_sock.close()

    time.sleep(1.0)
