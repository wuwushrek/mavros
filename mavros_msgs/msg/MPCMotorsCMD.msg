uint8 HORIZON_MPC = 10
uint8 MPC_ON = 1
uint8 MPC_OFF = 0
uint8 MPC_TEST = 2
uint8 MPC_RESET = 3

uint64 time_usec
uint64 time_init
float32 dt

float32[10] tb
float32[10] mx
float32[10] my
float32[10] mz
uint8 mpc_on
float32 tn