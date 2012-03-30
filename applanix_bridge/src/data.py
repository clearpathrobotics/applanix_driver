
# ROS
import rospy

# ROS messages
import applanix_bridge.msg as common
from applanix_ctl.msg import AllMsgs

# Node source
from port import Port
from mapping import groups, msgs
from handlers import GroupHandler, MessageHandler
import translator

# Python
from cStringIO import StringIO


class DataPort(Port):
  ALLMSGS_SEND_TIMEOUT = rospy.Duration.from_sec(0.01)

  def run(self):
    self.sock.settimeout(1.0)

    # Aggregate message for republishing the sensor config as a single blob.
    all_msgs = AllMsgs()
    all_msgs_pub = rospy.Publisher("config", AllMsgs, latch=True)
  
    # Set up handlers for translating different Applanix messages as they arrive.
    handlers = {}
    for group_num in groups.keys():
      include = True
      for prefix in self.opts['exclude_prefixes']:
        if groups[group_num][0].startswith(prefix): include = False
      if include:
        handlers[(common.CommonHeader.START_GROUP, group_num)] = GroupHandler(*groups[group_num])
    for msg_num in msgs.keys():
      handlers[(common.CommonHeader.START_MESSAGE, msg_num)] = MessageHandler(*msgs[msg_num], all_msgs=all_msgs)

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
