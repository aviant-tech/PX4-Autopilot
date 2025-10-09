uint64 timestamp                     # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id                     # unique device ID for the sensor that does not change between power cycles

float32 differential_pressure_pa     # differential pressure reading in Pascals (may be negative)

float32 temperature                  # Temperature provided by sensor in degrees Celsius, NAN if unknown

uint32 error_count                   # Number of errors detected by driver

uint32 corrupted_msgs_count          # Total number of messages detected with corrupted bits
uint32 accepted_corrupted_msgs_count # Number of messages let through with corrupted bits
