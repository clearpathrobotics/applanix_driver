#!/usr/bin/python

import roslib; roslib.load_manifest('applanix_ctl')
from roslib.packages import get_pkg_dir
from mapping import groups, msgs
from os import path

pkg_dir = get_pkg_dir('applanix_ctl')

all_msgs_filename = path.join(pkg_dir, "msg", "AllMsgs.msg")
with open(all_msgs_filename, "w") as all_msgs_f:
  for msg_num in msgs.keys():
    msg_tuple = msgs[msg_num]
    if msg_tuple:
      name, msg_cls = msg_tuple
      all_msgs_f.write("applanix_msgs/%s %s\n" % (msg_cls.__name__, name))

      msg_srv_filename = path.join(pkg_dir, "srv", "%s.srv" % msg_cls.__name__)
      print msg_cls.__name__mak#msg_srv_filename
      with open(msg_srv_filename, "w") as msg_srv_f:
        msg_srv_f.write("applanix_msgs/%s %s\n" % (msg_cls.__name__, name))
        msg_srv_f.write("---\n")
        msg_srv_f.write("applanix_msgs/Ack ack")
