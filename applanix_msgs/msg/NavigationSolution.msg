TimeDistance td

# Position in degrees
float64 latitude
float64 longitude

# Meters
float64 altitude

# Meters/second
float32 north_vel
float32 east_vel
float32 down_vel

# Degrees
float64 roll
float64 pitch
float64 heading
float64 wander_angle
float32 track_angle

# Meters/second
float32 speed

# Degrees/second
float32 ang_rate_long
float32 ang_rate_trans
float32 ang_rate_down

# Meters/second^2
float32 long_accel
float32 trans_accel
float32 down_accel

uint8 ALIGNMENT_FULL_NAVIGATION=0
uint8 ALIGNMENT_FINE_ALIGNMENT_ACTIVE=1
uint8 ALIGNMENT_GC_CHI_2=2
uint8 ALIGNMENT_PC_CHI_2=3
uint8 ALIGNMENT_GC_CHI_1=4
uint8 ALIGNMENT_PC_CHI_1=5
uint8 ALIGNMENT_COARSE_LEVELING=6
uint8 ALIGNMENT_INITIAL_SOLUTION=7
uint8 ALIGNMENT_NO_VALID_SOLUTION=8
uint8 alignment_status
