#!/usr/bin/env python

# ROS
import rospy
import roslib.message
import applanix_msgs.msg as msg
import applanix_ctl.srv as srv

response_codes = dict([(val, name) for name, val in msg.Ack.__dict__.items() if name.startswith("RESPONSE_")])


def main():
  com_ports = rospy.get_param('com_ports', None) 
  if com_ports != None:
    req_msg = msg.COMPortSetup()
    for port_num, port_params in enumerate(com_ports):
      # 8N1 is pretty universal now; less need to parameterize it. 
      port_msg = msg.COMPortParams()
      port_msg.parity = port_msg.PARITY_NONE
      port_msg.data_stop = port_msg.DATA_8_STOP_1
      port_msg.flow = port_msg.FLOW_NONE
      port_msg.baud = getattr(port_msg, "BAUD_%s" % port_params['baud'])
      port_msg.input_select = getattr(port_msg, "INPUT_%s" % port_params['input'])
      port_msg.output_select = getattr(port_msg, "OUTPUT_%s" % port_params['output'])
      req_msg.port_mask |= 2 << port_num
      req_msg.ports.append(port_msg)

    call_applanix_service("com_port_setup", req_msg)
    rospy.loginfo("Configured COM ports.")

  # Default rate of 10Hz
  rate = rospy.get_param('rate', 10)

  rospy.spin()


def call_applanix_service(name, req):
  service_defn = getattr(srv, req.__class__.__name__)
  rospy.wait_for_service(name)
  ack = rospy.ServiceProxy(name, service_defn)(req).ack
  if ack.response_code != msg.Ack.RESPONSE_ACCEPTED:
    rospy.logwarn("Parameter change call to %s resulted in error code %d (%s)." %
                  (name, ack.response_code, response_codes[ack.response_code]))
  return ack
