uint16 transaction

# TODO: enums
uint16 TYPE_NONE=0
uint16 TYPE_RTCM_1=1
uint16 TYPE_RTCM_3=2
uint16 TYPE_CMR=3
uint16 base_gnss_input_type

uint8 line_control

uint8 modem_control

uint8 connection_control

uint8[32] phone_number

uint8 num_redials

uint8[64] modem_command_string

uint8[128] modem_init_string

uint16 data_timeout_length

uint16 DATUM_WGS84=0
uint16 DATUM_NAD83=1
uint16 datum
