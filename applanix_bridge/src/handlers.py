from translator import Translator
from applanix_msgs.msg import Ack
import rospy


class Handler(object):
  def handle(self, data):
    raise NotImplementedError


class NullHandler(Handler):
  def handle(self, data):
    pass


class GroupHandler(Handler):
  def __init__(self, name, data_class):
    self.publisher = rospy.Publisher(name, data_class)
    self.msg = data_class()
    self.translator = Translator.for_msg(self.msg)

  def handle(self, buff):
    self.translator.deserialize(buff)
    self.publisher.publish(self.msg)


class MessageHandler(Handler):
  def __init__(self, name, data_class, all_msgs):
    self.name = name
    if data_class.in_all_msgs:
      self.msg = getattr(all_msgs, name) 
    else:
      self.msg = data_class()

    # Keep a reference to the all_msgs aggregate message for updating timestamp.
    self.all_msgs = all_msgs

  def handle(self, data):
    Translator.for_msg(self.msg).deserialize(data)
    self.all_msgs.last_changed = rospy.get_rostime()


class AckHandler(Handler):
  def __init__(self):
    self.msg = Ack()
    self.translator = Translator.for_msg(self.msg)

  def handle(self, buff):
    self.translator.deserialize(buff)
