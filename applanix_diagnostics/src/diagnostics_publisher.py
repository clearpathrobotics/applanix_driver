
# ROS
import rospy
import applanix_msgs.msg
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class BitfieldRepublisher(object):
  def __init__(self, topic_name, topic_type, fields):
    flags = []
    attrs = dir(applanix_msgs.msg.GeneralStatus)
    for field in fields:
      prefix = field.upper() + "_"
      field_flags = []
      for attr in attrs:
        if attr.startswith(prefix):
          short_name = attr[len(prefix):len(attr)]
          mask = getattr(applanix_msgs.msg.GeneralStatus, attr)
          field_flags.append((mask, short_name))
      field_flags.sort()
      flags.append((field, tuple(field_flags)))
    self.flags = tuple(flags)

    self.status_msg = DiagnosticArray()
    self.status_msg.status.append(DiagnosticStatus(
      level = DiagnosticStatus.OK,
      name = "Applanix AP10",
      message = "OK"))
    rospy.Subscriber(topic_name, topic_type, self._cb)
    self.pub = rospy.Publisher("/diagnostics", DiagnosticArray, latch=True)

  def _cb(self, msg):
    self.status_msg.status[0].values = []
    for field, field_flags in self.flags:
      field_value = getattr(msg, field)
      for mask, flag in field_flags:
        value = str(int((field_value & mask) != 0))
        self.status_msg.status[0].values.append(KeyValue(flag, value))
    self.pub.publish(self.status_msg) 


def main():
  rospy.init_node('applanix_diagnostics')
  BitfieldRepublisher("status/general", applanix_msgs.msg.GeneralStatus, \
                      ('status_a', 'status_b', 'status_c', 'fdir_1', \
                       'fdir_2', 'fdir_3', 'fdir_4', 'fdir_5', 'extended'))
  rospy.spin()
