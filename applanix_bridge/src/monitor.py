
import rospy
import threading


class Monitor(threading.Thread):
  def __init__(self, ports):
    super(Monitor, self).__init__()
    self.ports = ports
    self.finish = threading.Event()

  def run(self):
    while not self.finish:
      rospy.sleep(1.0)
      for name, port in ports.items():
        if not port.is_alive():
          rospy.logerr("Port %s thread died. Signalling node shutdown." % name)
          rospy.signal_shutdown("Node lost thread.")
