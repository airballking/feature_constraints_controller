# string identifier for the controller used
string controller_id
# hash identifier for this motion
int64 movement_id
# specify position ranges for the current constraint set
# which was specified using ConstraintConfig.msg
# NOTE: arrays in this and that message all need to have
# same size
float64[] pos_lo
float64[] pos_hi
float64[] weight
float64[] max_vel
float64[] min_vel
