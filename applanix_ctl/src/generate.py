#!/usr/bin/python

import roslib; roslib.load_manifest('applanix_ctl')
from roslib.packages import get_pkg_dir
from mapping import groups, msgs
from os import path
from sys import stdout

pkg_dir = get_pkg_dir('applanix_ctl')
output = []

all_msgs_filename = path.join(pkg_dir, "msg", "AllMsgs.msg")
with open(all_msgs_filename, "w") as all_msgs_f:
  all_msgs_f.write("time last_changed\n")
  all_msgs_f.write("time last_sent\n")
  for msg_num in msgs.keys():
    msg_tuple = msgs[msg_num]
    if msg_tuple:
      name, msg_cls, opts = msg_tuple
      if "no_receive" not in opts:
        all_msgs_f.write("applanix_msgs/%s %s\n" % (msg_cls.__name__, name))

      if name != "ack":
        msg_srv_filename = path.join(pkg_dir, "srv", "%s.srv" % msg_cls.__name__)
        output.append("%s.srv" % msg_cls.__name__)
        with open(msg_srv_filename, "w") as msg_srv_f:
          msg_srv_f.write("applanix_msgs/%s %s\n" % (msg_cls.__name__, name))
          msg_srv_f.write("---\n")
          msg_srv_f.write("applanix_msgs/Ack ack")

stdout.write(";".join(output))
