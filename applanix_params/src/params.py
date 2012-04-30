#!/usr/bin/env python

# ROS
import rospy
import roslib.message
import applanix_msgs.msg
import applanix_generated_msgs.srv 

response_codes = dict([(val, name) for name, val in applanix_msgs.msg.Ack.__dict__.items() if name.startswith("RESPONSE_")])


def main():
  rospy.init_node("applanix_params")

  com_ports = rospy.get_param('com_ports', None) 
  if com_ports != None:
    req_msg = applanix_msgs.msg.COMPortSetup()
    for port_num, port_params in enumerate(com_ports):
      # 8N1 is pretty universal now; less need to parameterize it. 
      port_msg = applanix_msgs.msg.COMPortParams()
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

  base_gnss = rospy.get_param('base_gnss', None) 
  if base_gnss != None:
    for base_num, base_params in enumerate(base_gnss):
      base_msg = applanix_msgs.msg.BaseGNSSSetup()
      base_msg.base_gnss_input_type = getattr(base_msg, "TYPE_%s" % base_params['type'])	
      base_msg.datum = getattr(base_msg, "DATUM_%s" % base_params['datum'])	
      call_applanix_service("base_gnss_%i_setup" % (base_num + 1), base_msg)
      rospy.loginfo("Configured base GNSS #%i." % (base_num + 1))

  geometry = rospy.get_param('geometry', None) 
  if geometry != None:
    req_msg = applanix_msgs.msg.GeneralParams()
    for vector_name in ['imu_lever_arm', 'primary_gnss_lever_arm', 'imu_mounting_angle', 'ref_mounting_angle']:
      if vector_name in geometry:
        vector = geometry[vector_name]
        getattr(req_msg, vector_name).x = vector['x']
        getattr(req_msg, vector_name).y = vector['y']
        getattr(req_msg, vector_name).z = vector['z']
    req_msg.time_types = 0x1
    req_msg.distance_type = 1
    req_msg.autostart = 1 
    req_msg.multipath = 2 
    call_applanix_service('general', req_msg)
    rospy.loginfo("Configured geometry.")

  # Default rate of 10Hz
  rate = rospy.get_param('rate', 10)
  rospy.Subscriber("subscribed_groups", applanix_msgs.msg.Groups, groups_callback)

  rospy.spin()


def call_applanix_service(name, req):
  service_defn = getattr(applanix_generated_msgs.srv, req.__class__.__name__)
  rospy.wait_for_service(name)
  ack = rospy.ServiceProxy(name, service_defn)(req).ack
  if ack.response_code != applanix_msgs.msg.Ack.RESPONSE_ACCEPTED:
    rospy.logwarn("Parameter change call to %s resulted in error code %d (%s)." %
                  (name, ack.response_code, response_codes[ack.response_code]))
  return ack


def groups_callback(message):
  req_msg = applanix_msgs.msg.PortControl()
  req_msg.rate = rospy.get_param('rate', 10)
  groups = set(message.groups)
  groups.add(10)
  for group_num in groups:
    req_msg.groups.append(applanix_msgs.msg.OutputGroup(group=group_num))
  call_applanix_service("primary_data_port", req_msg)
  #print message.groups
