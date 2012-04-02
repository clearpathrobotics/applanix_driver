
# ROS
import rospy

# ROS messages and services
import applanix_ctl.srv as srv
import applanix_msgs.msg as msg

# Node
from port import Port
from mapping import groups, msgs
from handlers import AckHandler

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

    for msg_num in msgs.keys():
      self.services.append(ServiceHandler(msg_num, self))
    self.services_ready.set()
    
    # Send the navigation mode every n seconds so that the Applanix device
    # doesn't close the connection on us.
    set_nav_mode = rospy.ServiceProxy("nav_mode", srv.NavModeControl)
    nav_mode_msg = applanix_msgs.msg.NavModeControl(mode = applanix_msgs.msg.NavModeControl.MODE_NAVIGATE)
    while not self.finish.is_set():
      rospy.sleep(SILENCE_INTERVAL)
      set_nav_mode(nav_mode_msg)

  def next_transaction(self):
    self.last_transaction_number += 1
    return self.last_transaction_number


class ServiceHandler(object):
  def __init__(self, msg_num, port):
    self.name, data_class = msgs[msg_num]
    self.port = port
    self.service = rospy.Service(self.name, getattr(srv, data_class.__name__), self.handle)

    # Part of the outbound message to Applanix device.
    self.header = msg.CommonHeader(start=msg.CommonHeader.START_MESSAGE, id=msg_num, length=0)

  def handle(self, req):
    # Called on the service's own thread, so acquire lock before using control socket.
    # Hold lock until entire interaction is complete, so that we guarantee the next message
    # received is our ack.
    with self.port.lock:
      # Write request to self.sock
      msg = getattr(req, self.name)
      msg.transaction = self.port.next_transaction() 
      self.port.send(self.header, msg) 

      # Read response from port, return it.
      pkt_id, pkt_str = self.port.recv()
      
      if pkt_id == None:
        raise ValueError("No response message on control port.")

      if pkt_id != (msg.CommonHeader.START_MESSAGE, 0):
        raise ValueError("Non-ack message on control port: %s.%d" % pkt_id)

      handler = AckHandler()
      handler.handle(StringIO(pkt_str))

      return handler.msg
