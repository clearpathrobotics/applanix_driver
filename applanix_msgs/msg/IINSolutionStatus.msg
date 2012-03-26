# Group 20
TimeDistance td

uint16 num_satellites
float32 a_priori_pdop
float32 baseline_length

uint16 STATUS_FIXED_NARROW_LANE_RTK=1
uint16 STATUS_FIXED_WIDE_LANE_RTK=2
uint16 STATUS_FLOAT_RTK=3
uint16 STATUS_CODE_DGPS=4
uint16 STATUS_RTCM_DGPS=5
uint16 STATUS_AUTONOMOUS_CA=6
uint16 STATUS_GNSS_NAV_SOLUTION=7
uint16 STATUS_NO_SOLUTION=8
uint16 status

uint8[12] prn_assignment

uint16 l1_cycle_slip_flag 
uint16 l2_cycle_slip_flag 
