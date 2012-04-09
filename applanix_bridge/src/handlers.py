from translator import Translator
from applanix_msgs.msg import Ack
import rospy

from applanix_msgs.msg import Ack


class Handler(object):
  def handle(self, data):
    raise NotImplementedError


class NullHandler(Handler):
  def handle(self, data):
    pass


class GroupHandler(Handler):
  def __init__(self, name, data_class, listener):
    self.publisher = rospy.Publisher(name, data_class, subscriber_listener=listener)
    self.message = self.publisher.data_class()

  def handle(self, buff):
    self.message.translator().deserialize(buff)
    self.publisher.publish(self.message)


class MessageHandler(Handler):
  def __init__(self, name, data_class, all_msgs):
    self.name = name
    if data_class.in_all_msgs:
      self.message = getattr(all_msgs, name) 
    else:
      self.message = data_class()

    # Keep a reference to the all_msgs aggregate message for updating timestamp.
    self.all_msgs = all_msgs

  def handle(self, buff):
    self.message.translator().deserialize(buff)
    self.all_msgs.last_changed = rospy.get_rostime()


class AckHandler(Handler):
  def __init__(self):
    self.message = Ack()

  def handle(self, buff):
    self.message.translator().deserialize(buff)

