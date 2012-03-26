# Msg 20
uint16 transaction

uint8 time_types
uint8 distance_type
uint8 autostart

geometry_msgs/Point32 imu_lever_arm
geometry_msgs/Point32 primary_gnss_lever_arm
geometry_msgs/Point32 aux_1_gnss_lever_arm
geometry_msgs/Point32 aux_2_gnss_lever_arm
geometry_msgs/Point32 imu_mounting_angle
geometry_msgs/Point32 ref_mounting_angle

uint8 MULTIPATH_LOW=0
uint8 MULTIPATH_MEDIUM=1
uint8 MULTIPATH_HIGH=2
uint8 multipath
