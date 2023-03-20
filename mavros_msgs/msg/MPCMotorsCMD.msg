uint8 MPC_ON = 1
uint8 MPC_OFF = 0
uint8 MPC_TEST = 2
uint8 MPC_RESET = 3

uint64 time_usec

float32[6] motor_val_des
float32[4] thrust_and_angrate_des

uint8 mpc_on
uint8 weight_motors