std_msgs/Header header

uint8 STATE_UNINITIALIZED=0
uint8 STATE_VALID=1
uint8 STATE_INVALID_WKC=2
uint8 STATE_FAULT_LATCHED=3
uint8 STATE_SHUTDOWN=4

uint64 cycle_sequence
uint64 source_cycle_sequence
uint64 last_invalid_cycle_sequence
uint64 monotonic_timestamp_ns
uint64 data_age_ns
uint64 total_invalid_cycles
uint32 consecutive_invalid_cycles
int32 actual_wkc
int32 expected_wkc
uint8 cycle_state
bool data_valid
bool has_valid_data
bool fault_latched

float64[23] position
float64[23] velocity
float64[23] torque
