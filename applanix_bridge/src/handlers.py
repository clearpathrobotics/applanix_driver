import translator
import rospy


class Handler:
  def handle(self, data):
    raise NotImplementedError


class NullHandler(Handler):
  def handle(self, data):
    pass


class GroupHandler(Handler):
  def __init__(self, name, data_class):
    self.translator = translator.get(data_class)
    self.publisher = rospy.Publisher(name, data_class)

  def handle(self, data):
    msg = self.publisher.data_class()
    self.translator.deserialize(data, msg)
    self.publisher.publish(msg)


class MessageHandler(Handler):
  def __init__(self, name, data_class, all_msgs):
    self.name = name
    if hasattr(data_class, 'no_receive') and data_class.no_receive == True:
      self.msg = data_class()
    else:
      self.msg = getattr(all_msgs, name) 
    self.translator = translator.get(data_class)

    # Keep a reference to the all_msgs aggregate message.
    self.all_msgs = all_msgs

  def handle(self, data):
    self.translator.deserialize(data, self.msg)
    self.all_msgs.last_changed = rospy.get_rostime()


class AckHandler(Handler):
  pass
