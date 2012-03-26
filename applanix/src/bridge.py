#!/usr/bin/env python

import roslib, rospy, roslib.genpy
import applanix.msg as common
from mapping import groups, msgs

from pprint import pprint
import socket
import struct
from cStringIO import StringIO

import translator
from handlers import NullHandler, GroupHandler, MessageHandler, AckHandler

PORT_LOGGING = 5603
PORT_REALTIME = 5602
PORT_CONTROL = 5601

DEFAULT_IP = '10.25.0.51'
PREFIX_DEFAULTS = {
    "raw": True,
    "dmi": True,
    "status": True,
    "events": True
    }


def main():
  rospy.init_node('applanix_bridge')
  exclude_prefixes = []
  ip = rospy.get_param('~ip', DEFAULT_IP)
  for prefix, default in PREFIX_DEFAULTS.items():
    if not rospy.get_param('~include_%s' % prefix, default):
      exclude_prefixes.append(prefix)

  handlers = {}
  null_handler = NullHandler()

  def mk_group_handler(group_tuple):
    if not group_tuple:
      return null_handler
    topic, msg_cls = group_tuple
    for prefix in exclude_prefixes:
      if topic.startswith(prefix): 
        return null_handler
    return GroupHandler(topic, msg_cls)

  for group_num in groups.keys():
    handlers[(common.CommonHeader.START_GROUP, group_num)] = mk_group_handler(groups[group_num])

  def mk_msg_handler(msg_tuple):
    name, msg_cls = msg_tuple
    return MessageHandler(name, msg_cls)

  for msg_num in msgs.keys():
    handlers[(common.CommonHeader.START_MESSAGE, msg_num)] = mk_msg_handler(msgs[msg_num])


  ip_port = (ip, PORT_REALTIME)
  sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  sock.connect(ip_port)
  sock.settimeout(2.0)
  buff = sock.makefile()

  counters = {}
  bad = set()
  header_translator = translator.get(common.CommonHeader)

  while not rospy.is_shutdown(): 
    header = common.CommonHeader()
    header_translator.deserialize(buff, header)
    pkt = (str(header.start), header.id)

    data = StringIO(sock.recv(header.length))

    try:
      #if header.id == 12:
      #  print msg
      #  print header.length
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
      if pkt[0] == '$GRP' and pkt[1] >= 20000:
        # Internal Applanix diagnostic messages.
        warn = False
      if warn:
        rospy.logwarn("Unimplemented %s.%d" % pkt)

    if pkt not in counters:
      counters[pkt] = 0
    counters[pkt] += 1
