# Msg 51, 52, 61
uint16 transaction

uint16 groups_count
OutputGroup[] groups

# 1, 2, 10, 20, 25, 50, 100, 200
uint16 rate

uint8 AUTOLOG_DISABLED=0
uint8 AUTOLOG_ENABLED=1
uint8 autolog

uint8 DISK_LOGGING_STOP=0
uint8 DISK_LOGGING_START=1
uint8 disk_logging

uint8[32] filename_kernel
uint8[32] reserved
