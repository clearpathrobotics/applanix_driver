
# ROS
import rospy

# ROS messages and services
import applanix_bridge.msg as common
import applanix_ctl.srv as srv

# Node
from port import Port
from mapping import groups, msgs
from handlers import AckHandler

# Python
from threading import Lock
from cStringIO import StringIO


class ControlPort(Port):
  def run(self):
    services = []
    self.lock = Lock()
    self.last_transaction_number = 0

    for msg_num in msgs.keys():
      services.append(ServiceHandler(msg_num, self))

    while not self.finish.is_set():
      rospy.sleep(1.0)

  def next_transaction(self):
    self.last_transaction_number += 1
    return self.last_transaction_number


class ServiceHandler(object):
  def __init__(self, msg_num, port):
    self.name, data_class = msgs[msg_num]
    self.port = port
    self.service = rospy.Service(self.name, getattr(srv, data_class.__name__), self.handle)

    # Part of the outbound message to Applanix device.
    self.header = common.CommonHeader(start=common.CommonHeader.START_MESSAGE, id=msg_num, length=0)

  def handle(self, req):
    # Called on the service's own thread, so acquire lock before using control socket.
    # Hold lock until entire interaction is complete, so that we guarantee the next message
    # received is our ack.
    with self.port.lock:
      # Write request to self.sock
      msg = getattr(req, self.name)
      msg.transaction = self.port.next_transaction() 
      self.port.send(self.header, msg) 

      # Read response from, return it.
      pkt_id, pkt_str = self.port.recv()
      
      if pkt_id != (common.CommonHeader.START_MESSAGE, 0):
        raise ValueError("Non-ack message on control port.")

      handler = AckHandler()
      handler.handle(StringIO(pkt_str))

      return handler.msg
