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
ALLMSGS_SEND_TIMEOUT = rospy.Duration.from_sec(0.01)


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
    data_sock.settimeout(0.1)
    ip_port = (ip, PORTS_DATA[data_port])
    data_sock.connect(ip_port)
    rospy.loginfo("Successfully connected to data port at %s:%d" % ip_port)
  except socket.error:
    rospy.logfatal("Couldn't connect to data port at %s:%d." % ip_port)
    exit(1)

  control_sock = None
  if control:
    try:
      control_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      ip_port = (ip, PORT_CONTROL)
      control_sock.connect(ip_port)
      rospy.loginfo("Successfully connected to control port at %s:%d" % ip_port)
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
  header = common.CommonHeader()
  footer = common.CommonFooter()
  translator.get(common.CommonHeader)
  translator.get(common.CommonFooter)
  checksum_struct = struct.Struct("<hh")

  try:
    while not rospy.is_shutdown():
      
      """skipped_chars = []
      initial_chars = []
      while True:
        initial_chars.append(data_sock.recv(1))
        if initial_chars == ["$", "G"] or initial_chars == ["$", "M"]:
          break
        elif initial_chars != ["$"]: 
          skipped_chars.extend(initial_chars)
          initial_chars = []
      if len(skipped_chars) > 0:
        rospy.logwarn("Skipped %d out-of-message characters on wire." % len(skipped_chars))
      """
      # Receive remaining header from data socket.
      # header_str = ''.join(initial_chars) + data_sock.recv(header.translator.size - len(initial_chars))
      header_str = data_sock.recv(header.translator.size)
      header_data = StringIO(header_str)
      header.translator.deserialize(header_data, header)
      pkt = (str(header.start).encode('string_escape'), header.id)

      # Initial sanity check.
      if pkt[0] not in (common.CommonHeader.START_GROUP, common.CommonHeader.START_MESSAGE):
        rospy.logwarn("Bad header %s.%d" % pkt)
        continue

      # Special case for a troublesome undocumented packet.
      # It reports a length of 132, but contains no checksum or footer, and is trailed
      # by three out-of-message bytes.
      if pkt == ("$GRP", 20015):
        data_sock.recv(135)
        continue

      # Receive remainder of packet from data socket. 
      packet_str = data_sock.recv(header.length)

      # Check footer and checksum.
      footer_data = StringIO(packet_str[-footer.translator.size:])
      footer.translator.deserialize(footer_data, footer)
      if str(footer.end) != common.CommonFooter.END:
        rospy.logwarn("Bad footer from packet %s.%d" % pkt)
        continue

      checksum = 0
      packet_data = StringIO(header_str + packet_str)
      while True:
        data = packet_data.read(checksum_struct.size)
        if data == "": break
        c1, c2 = checksum_struct.unpack(data)
        checksum += c1 + c2
      if checksum % 65536 != 0:
        rospy.logwarn("Bad checksum from packet %s.%d: %%d" % pkt % checksum)
        continue

      try:
        payload_data = StringIO(packet_str)
        handlers[pkt].handle(payload_data)

      except translator.TranslatorError:
        if pkt not in bad:
          rospy.logwarn("Error parsing %s.%d" % pkt)
          raise
          #bad.add(pkt)

      except KeyError:
        warn = True
        if pkt in counters:
          # Already seen this message-- only warn once.
          warn = False
        if pkt[1] >= 20000:
          # Undocumented messages.
          warn = False
        if warn:
          rospy.logwarn("Unimplemented %s.%d" % pkt)

      if pkt not in counters:
        counters[pkt] = 0
      counters[pkt] += 1

      if all_msgs.last_changed > all_msgs.last_sent and \
          rospy.get_rostime() > all_msgs.last_changed + ALLMSGS_SEND_TIMEOUT:
        all_msgs_pub.publish(all_msgs)
        # print "PUBLISH!!!"
        all_msgs.last_sent = rospy.get_rostime()

  except socket.error as e:
    # Swallow a standard interruption error.
    if (e.errno != 4):
      raise

  finally:
    if control_sock:
      rospy.loginfo("Closing control socket")
      control_sock.close()

    rospy.loginfo("Closing data socket")
    data_sock.close()

    time.sleep(1.0)
