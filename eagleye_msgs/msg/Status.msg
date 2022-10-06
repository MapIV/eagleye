# error code
uint8 NAN_OR_INFINITE=0
uint8 TOO_LARGE_OR_SMALL=1

bool enabled_status # It's true in the IMU accumulation state, indicating available status.
bool estimate_status # true if the value is updated by GNSS, etc.
bool is_abnormal
uint8 error_code
