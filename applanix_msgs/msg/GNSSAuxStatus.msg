TimeDistance td

uint8 SOLUTION_UNKNOWN=255
uint8 SOLUTION_NO_DATA=0
uint8 SOLUTION_HORIZONTAL_CA=1
uint8 SOLUTION_3D_CA=2
uint8 SOLUTION_HORIZONTAL_DGPS=3
uint8 SOLUTION_3D_DGPS=4
uint8 SOLUTION_FLOAT_RTK=5
uint8 SOLUTION_WIDE_LANE_RTK=6
uint8 SOLUTION_NARROW_LANE_RTK=7
uint8 SOLUTION_P_CODE=8
uint8 SOLUTION_OMNISTAR_HP=9
uint8 SOLUTION_OMNISTAR_XP=10
uint8 SOLUTION_OMNISTAR_VBS=11
uint8 solution_status

uint8 channels_count
uint16 reserved
GNSSChannelStatus[] channels

float32 hdop
float32 vdop
float32 dgps_latency
uint16 dgps_reference_id

uint32 gps_week
float64 gps_time_offset

float32 gnss_latency
float32 geoidal_separation

#TODO: enum
uint16 nmea_messages_received
uint8 in_use
