
#include "pid.h"
////////////////////////////////////////////////////////////////////////////////////////////////
float pid_roll_setpoint = 0;
float pid_pitch_setpoint = 0;
float pid_yaw_setpoint = 0;

float pid_error_temp = 0;
float pid_i_mem_roll = 0;
float pid_i_mem_pitch = 0;
float pid_i_mem_yaw = 0;
float pid_output_roll = 0;
float pid_output_pitch = 0;
float pid_output_yaw = 0;
float pid_last_roll_d_error = 0;
float pid_last_pitch_d_error = 0;
float pid_last_yaw_d_error = 0;

float gyro_roll_input = 0;
float gyro_pitch_input = 0;
float gyro_yaw_input = 0;

float pid_p_gain_roll = 1.3;  // 1.3
float pid_i_gain_roll = 0.04; // 0.04
float pid_d_gain_roll = 18.0; // 18.0
int pid_max_roll = 400;       // Maximum output of the PID-controller (+/-).

float pid_p_gain_pitch = pid_p_gain_roll; // Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll; // Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll; // Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;         // Maximum output of the PID-controller (+/-).

float pid_p_gain_yaw = 4.0;  // Gain setting for the pitch P-controller (default = 4.0).
float pid_i_gain_yaw = 0.02; // Gain setting for the pitch I-controller (default = 0.02).
float pid_d_gain_yaw = 0.0;  // Gain setting for the pitch D-controller (default = 0.0).
int pid_max_yaw = 400;       // Maximum output of the PID-controller (+/-).
float pid_i_max = 200;

void calculate_pid(void)
{
    // Roll calculations
    pid_error_temp = gyro_roll_input - pid_roll_setpoint;
    pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
    if (pid_i_mem_roll > pid_i_max)
        pid_i_mem_roll = pid_i_max;
    else if (pid_i_mem_roll < pid_i_max * -1)
        pid_i_mem_roll = pid_i_max * -1;

    pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
    if (pid_output_roll > pid_max_roll)
        pid_output_roll = pid_max_roll;
    else if (pid_output_roll < pid_max_roll * -1)
        pid_output_roll = pid_max_roll * -1;

    pid_last_roll_d_error = pid_error_temp;

    // Pitch calculations
    pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
    pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
    if (pid_i_mem_pitch > pid_i_max)
        pid_i_mem_pitch = pid_i_max;
    else if (pid_i_mem_pitch < pid_i_max * -1)
        pid_i_mem_pitch = pid_i_max * -1;

    pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
    if (pid_output_pitch > pid_max_pitch)
        pid_output_pitch = pid_max_pitch;
    else if (pid_output_pitch < pid_max_pitch * -1)
        pid_output_pitch = pid_max_pitch * -1;

    pid_last_pitch_d_error = pid_error_temp;

    // Yaw calculations
    pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
    pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
    if (pid_i_mem_yaw > pid_i_max)
        pid_i_mem_yaw = pid_i_max;
    else if (pid_i_mem_yaw < pid_i_max * -1)
        pid_i_mem_yaw = pid_i_max * -1;

    pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
    if (pid_output_yaw > pid_max_yaw)
        pid_output_yaw = pid_max_yaw;
    else if (pid_output_yaw < pid_max_yaw * -1)
        pid_output_yaw = pid_max_yaw * -1;

    pid_last_yaw_d_error = pid_error_temp;
}
