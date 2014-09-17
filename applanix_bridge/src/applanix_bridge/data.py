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

# ROS
import rospy

# ROS messages
import applanix_msgs.msg

# Node source
from port import Port
from applanix_msgs.mapping import groups, msgs
from handlers import GroupHandler, MessageHandler
import translator

# Python
from cStringIO import StringIO
from threading import Lock


class DataPort(Port):
  ALLMSGS_SEND_TIMEOUT = rospy.Duration.from_sec(0.01)

  def run(self):
    self.sock.settimeout(1.0)
    errors = 0

    # Aggregate message for republishing the sensor config as a single blob.
    all_msgs = applanix_msgs.msg.AllMsgs()
    all_msgs_pub = rospy.Publisher("config", all_msgs.__class__, queue_size=5, latch=True) 

    # Listener object which tracks what topics have been subscribed to.
    listener = SubscribeListenerManager()
  
    # Set up handlers for translating different Applanix messages as they arrive.
    handlers = {}
    for group_num in groups.keys():
      include = True
      for prefix in self.opts['exclude_prefixes']:
        if groups[group_num][0].startswith(prefix): include = False
      if include:
        handlers[(applanix_msgs.msg.CommonHeader.START_GROUP, group_num)] = \
            GroupHandler(*groups[group_num], listener=listener.listener_for(group_num))
    for msg_num in msgs.keys():
      handlers[(applanix_msgs.msg.CommonHeader.START_MESSAGE, msg_num)] = \
          MessageHandler(*msgs[msg_num], all_msgs=all_msgs)

    pkt_counters = {}
    bad_pkts = set()

    while not self.finish.is_set():
      try:
        pkt_id, pkt_str = self.recv()
        if pkt_id != None:
          handlers[pkt_id].handle(StringIO(pkt_str))

      except ValueError as e:
        # Some problem in the recv() routine.
        rospy.logwarn(str(e))
        continue

      except KeyError:
        # No handler for this pkt_id. Only warn on the first sighting.
        if pkt_id not in pkt_counters:
          rospy.logwarn("Unhandled packet: %s.%d" % pkt_id)

      except translator.TranslatorError:
        if pkt_id not in bad_pkts:
          rospy.logwarn("Error parsing %s.%d" % pkt_id)
          bad_pkts.add(pkt)

      if pkt_id not in pkt_counters:
        pkt_counters[pkt_id] = 0
      pkt_counters[pkt_id] += 1


      # Since the config messages come all at once in a burst, we can time out in between those
      # bursts and send this aggregate message out with all of them at once.
      if all_msgs.last_changed > all_msgs.last_sent and \
          rospy.get_rostime() > all_msgs.last_changed + self.ALLMSGS_SEND_TIMEOUT:
        all_msgs_pub.publish(all_msgs)
        all_msgs.last_sent = rospy.get_rostime() 


class SubscribeListenerManager():
  def __init__(self):
    self.lock = Lock()
    self.groups = set()
    self.publisher = rospy.Publisher("subscribed_groups", applanix_msgs.msg.Groups, queue_size=5, latch=True)
    self.publish()

  def publish(self):
    self.publisher.publish(groups=list(self.groups))

  def listener_for(self, group_num):
    return self.Listener(self, group_num)

  class Listener(rospy.SubscribeListener):
    def __init__(self, manager, group_num):
      self.manager = manager
      self.group_num = group_num

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
      with self.manager.lock:
        self.manager.groups.add(self.group_num)
        self.manager.publish()

    def peer_unsubscribe(self, topic_name, num_peers):
      if num_peers == 0:
        with self.lock:
          self.manager.groups.discard(self.group_num)
          self.manager.publish()
