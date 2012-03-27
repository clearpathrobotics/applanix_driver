
import rospy

class DeferredPublisher(rospy.Publisher):
  def __init__(self, name, data_class, **kwargs):
    self.kwargs = kwargs
    self.name = name
    self.data_class = data_class
    self.initialized = False

  def init(self):
    super(DeferredPublisher, self).__init__(self.name, self.data_class, **self.kwargs)
    self.initialized = True 

  def publish(self, *args, **kwargs):
    if not self.initialized:
      self.init()
      #raise RuntimeError("Deferred publisher not initialized.")
    super(DeferredPublisher, self).publish(*args, **kwargs)
