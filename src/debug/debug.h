
#include "common.h"
extern unsigned long t;
extern unsigned long print_time, blink_time, time_function_blink;
// PWM Input values
extern volatile uint32_t pwm_values[6];
extern float desired_value[3];
extern float Throttle;
extern float error_inner[3];
extern float pid_inner[3];
extern int esc[4];
extern float gyr_fr_dps[3];