time stamp

# base (shared) values
int32 test_number
bool default_control
string flight_mode
bool test_finished
bool coord_ok

# PrimaryStatus
string control_mode  # did something break?

# Test Progress
## regulation
bool regulate_finished

## uc_bound for tube MPC
bool uc_bound_activated
bool uc_bound_finished

## mrpi for tube MPC
bool mrpi_finished

## tube MPC
bool traj_sent
bool traj_finished
int32 gain_mode

## lqrrrt
bool lqrrrt_activated
bool lqrrrt_finished

## info rich traj
bool info_traj_send  # used by info_rich_traj to start sending setpoints 

## DMPC
int32 solver_status
float32 cost_value
float32 kkt_value
float32 sol_time

## rattle
int32 rattle_use_params
int32 rattle_weight_mode
int32 rattle_initial_model_mode
bool activate_rattle #used only for test 13, for point B to C, then shut off

string description  # for telemetry description
