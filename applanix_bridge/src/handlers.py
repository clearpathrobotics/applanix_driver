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
    self.translator = translator.get(data_class)

    if data_class.in_all_msgs:
      self.msg = getattr(all_msgs, name) 
    else:
      self.msg = data_class()

    # Keep a reference to the all_msgs aggregate message for updating timestamp.
    self.all_msgs = all_msgs

  def handle(self, data):
    self.translator.deserialize(data, self.msg)
    self.all_msgs.last_changed = rospy.get_rostime()



class AckHandler(Handler):
  pass
