
#include "common.h"
// Manual accelerometer calibration values for IMU angles:
extern int16_t manual_acc_pitch_cal_value;
extern int16_t manual_acc_roll_cal_value;
extern uint8_t use_manual_calibration;
extern int16_t manual_gyro_pitch_cal_value;
extern int16_t manual_gyro_roll_cal_value;
extern int16_t manual_gyro_yaw_cal_value;
extern uint8_t gyro_address;
extern int16_t cal_int;
extern int16_t temperature;
extern int16_t acc_x, acc_y, acc_z;
extern int16_t gyro_pitch, gyro_roll, gyro_yaw;

extern int32_t acc_total_vector;
extern int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;

void gyro_setup(void);
void gyro_signalen(void);
void calibrate_gyro(void);