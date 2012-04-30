
# ROS
import rospy

# ROS messages and services
import applanix_generated_msgs.srv
import applanix_msgs.msg

# Node
from port import Port
from handlers import AckHandler
import mapping

# Python
import threading
from cStringIO import StringIO


SILENCE_INTERVAL=5.0


class ControlPort(Port):
  def run(self):
    self.sock.setblocking(1)
    self.services_ready = threading.Event()
    self.services = []
    self.lock = threading.Lock()
    self.last_transaction_number = 0

    for msg_num in mapping.msgs.keys():
      if msg_num != 0:
        self.services.append(ServiceHandler(msg_num, self))
    self.services_ready.set()
    
    # Send the navigation mode every n seconds so that the Applanix device
    # doesn't close the connection on us.
    set_nav_mode = rospy.ServiceProxy("nav_mode", applanix_generated_msgs.srv.NavModeControl)
    nav_mode_msg = applanix_msgs.msg.NavModeControl(mode=applanix_msgs.msg.NavModeControl.MODE_NAVIGATE)
    while not self.finish.is_set():
      rospy.sleep(SILENCE_INTERVAL)
      set_nav_mode(nav_mode_msg)

  def next_transaction(self):
    self.last_transaction_number += 1
    return self.last_transaction_number


class ServiceHandler(object):
  def __init__(self, msg_num, port):
    self.name, data_class = mapping.msgs[msg_num]
    self.port = port
    self.service = rospy.Service(self.name, getattr(applanix_generated_msgs.srv, data_class.__name__), self.handle)

    # Part of the outbound message to Applanix device.
    self.header = applanix_msgs.msg.CommonHeader(start=applanix_msgs.msg.CommonHeader.START_MESSAGE, id=msg_num, length=0)

  def handle(self, message):
    # Called on the service's own thread, so acquire lock before using control socket.
    # Hold lock until entire interaction is complete, so that we guarantee the next message
    # received is our ack.
    with self.port.lock:
      # Write request to self.sock
      message.request.transaction = self.port.next_transaction() 
      self.port.send(self.header, message.request)

      # Read response from port, return it.
      pkt_id, pkt_str = self.port.recv()
      
      if pkt_id == None:
        raise ValueError("No response message on control port.")

      if pkt_id != (applanix_msgs.msg.CommonHeader.START_MESSAGE, 0):
        raise ValueError("Non-ack message on control port: %s.%d" % pkt_id)

      handler = AckHandler()
      handler.handle(StringIO(pkt_str))

      return handler.message
