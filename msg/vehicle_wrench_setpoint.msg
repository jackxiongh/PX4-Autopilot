
uint64 timestamp        # time since system start (microseconds)
uint64 timestamp_sample # timestamp of the data sample on which this message is based (microseconds)

bool wrench_sp_valid    # only use this wrench setpoint if true

float32[6] wrench       # wrench setpoint [fx, fy, fz, tx, ty, tz]
