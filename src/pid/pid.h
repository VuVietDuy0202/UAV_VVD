
#include "common.h"

extern float pid_p_gain_roll;
extern float pid_i_gain_roll;
extern float pid_d_gain_roll;
extern int pid_max_roll;

extern float pid_p_gain_pitch;
extern float pid_i_gain_pitch;
extern float pid_d_gain_pitch;
extern int pid_max_pitch;

extern float pid_p_gain_yaw;
extern float pid_i_gain_yaw;
extern float pid_d_gain_yaw;
extern int pid_max_yaw;

extern float pid_roll_setpoint;
extern float pid_pitch_setpoint;
extern float pid_yaw_setpoint;

extern float pid_error_temp;
extern float pid_i_mem_roll;
extern float pid_i_mem_pitch;
extern float pid_i_mem_yaw;
extern float pid_output_roll;
extern float pid_output_pitch;
extern float pid_output_yaw;
extern float pid_last_roll_d_error;
extern float pid_last_pitch_d_error;
extern float pid_last_yaw_d_error;

// mpu
extern float gyro_roll_input;
extern float gyro_pitch_input;
extern float gyro_yaw_input;

void calculate_pid(void);
