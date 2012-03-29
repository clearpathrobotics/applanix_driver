#!/usr/bin/env python

# ROS
import rospy

# ROS messages and services.
import applanix_bridge.msg as common
from applanix_ctl.msg import AllMsgs
import applanix_ctl.srv as srv

# Package modules.
from mapping import groups, msgs
from checksum import checksum
from handlers import GroupHandler, MessageHandler
import translator

# Standard.
from pprint import pprint
from cStringIO import StringIO
from threading import Lock
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
  control_enabled = rospy.get_param('~control', True)

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

  def service_call(req):
    print req

  control_sock = None
  control_lock = Lock()
  services = {}
  if control_enabled:
    try:
      control_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      ip_port = (ip, PORT_CONTROL)
      #control_sock.connect(ip_port)
      rospy.loginfo("Successfully connected to control port at %s:%d" % ip_port)
 
      for msg_num in msgs.keys():
        name, data_class = msgs[msg_num]
        services[name] = rospy.Service(name, getattr(srv, data_class.__name__), service_call)
    except socket.error:
      rospy.logfatal("Couldn't connect to control port at %s:%d." % ip_port)
      data_sock.close()
      exit(1)


  # Aggregate message for republishing the sensor config as a single blob.
  all_msgs = AllMsgs()
  all_msgs_pub = rospy.Publisher("config", AllMsgs, latch=True)

  
  # Set up handlers for translating different Applanix messages as they arrive.
  handlers = {}
  for group_num in groups.keys():
    include = True
    for prefix in exclude_prefixes:
      if groups[group_num][0].startswith(prefix): include = False
    if include:
      handlers[(common.CommonHeader.START_GROUP, group_num)] = GroupHandler(*groups[group_num])
  for msg_num in msgs.keys():
    handlers[(common.CommonHeader.START_MESSAGE, msg_num)] = MessageHandler(*msgs[msg_num], all_msgs=all_msgs)


  pkt_counters = {}
  bad_pkts = set()
  header = common.CommonHeader()
  footer = common.CommonFooter()
  translator.get(common.CommonHeader)
  translator.get(common.CommonFooter)

  try:
    while not rospy.is_shutdown():
      # Receive header from data socket.
      header_str = data_sock.recv(header.translator.size)
      header_data = StringIO(header_str)
      header.translator.deserialize(header_data, header)
      pkt = (str(header.start).encode('string_escape'), header.id)

      try:
        # Initial sanity check.
        if pkt[0] not in (common.CommonHeader.START_GROUP, common.CommonHeader.START_MESSAGE):
          raise ValueError("Bad header %s.%d" % pkt)

        # Special case for a troublesome undocumented packet.
        if pkt == ("$GRP", 20015):
          data_sock.recv(135)
          continue

        # Receive remainder of packet from data socket. 
        packet_str = data_sock.recv(header.length)

        # Check package footer.
        footer_data = StringIO(packet_str[-footer.translator.size:])
        footer.translator.deserialize(footer_data, footer)
        if str(footer.end) != common.CommonFooter.END:
          raise("Bad footer from packet %s.%d" % pkt)

        # Check package checksum.
        if checksum(StringIO(header_str + packet_str)) != 0:
          raise("Bad checksum from packet %s.%d: %%d" % pkt % checksum)

      except ValueError as e:
        rospy.logwarn(str(e))
        continue


      try:
        handlers[pkt].handle(StringIO(packet_str))

      except KeyError:
        # Only warn on the first sighting.
        if pkt not in pkt_counters:
          rospy.logwarn("Unhandled data: %s.%d" % pkt)

      except translator.TranslatorError:
        if pkt not in bad_pkts:
          rospy.logwarn("Error parsing %s.%d" % pkt)
          bad_pkts.add(pkt)

      if pkt not in pkt_counters:
        pkt_counters[pkt] = 0
      pkt_counters[pkt] += 1

      if all_msgs.last_changed > all_msgs.last_sent and \
          rospy.get_rostime() > all_msgs.last_changed + ALLMSGS_SEND_TIMEOUT:
        all_msgs_pub.publish(all_msgs)
        all_msgs.last_sent = rospy.get_rostime()

    # end while

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

    rospy.sleep(1.0)
