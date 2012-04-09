TimeDistance td

#uint8[24] data
int32 delta_velocity_x
int32 delta_velocity_y
int32 delta_velocity_z

int32 delta_accel_x
int32 delta_accel_y
int32 delta_accel_z

# Bitmask
uint8 STATUS_1_BAD_FRAME=1
uint8 STATUS_2_BAD_FRAMES=2
uint8 STATUS_3_BAD_FRAMES=4
uint8 data_status

uint8 imu_type
uint8 imu_data_rate
uint16 imu_status
