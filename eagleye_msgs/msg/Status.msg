# error code
uint8 NAN_OR_INFINITE=0
uint8 TOO_LARGE_OR_SMALL=1

bool enabled_status # This flag will be true the first time the value is estimated, and will remain true thereafter.
bool estimate_status # This flag will be true only when the value is estimated.
bool is_abnormal
uint8 error_code