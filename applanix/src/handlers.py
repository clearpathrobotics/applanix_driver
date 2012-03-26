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
  def __init__(self, name, data_class):
    self.name = name
    self.translator = translator.get(data_class)

  def handle(self, data):
    print "Received:", name, " len:", len(data)


class AckHandler(Handler):
  pass
