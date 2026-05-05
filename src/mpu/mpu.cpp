#include "mpu.h"
// Manual accelerometer calibration values for IMU angles:
int16_t manual_acc_pitch_cal_value = 196;
int16_t manual_acc_roll_cal_value = 3;

uint8_t use_manual_calibration = false;
int16_t manual_gyro_pitch_cal_value = -14;
int16_t manual_gyro_roll_cal_value = 61;
int16_t manual_gyro_yaw_cal_value = 9;

uint8_t gyro_address = 0x68; // The I2C address of the MPU-6050 is 0x68 in hexadecimal form.
int16_t cal_int;
int16_t temperature;
int16_t acc_x, acc_y, acc_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;

int32_t acc_total_vector;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;

void gyro_setup(void)
{
    Wire.beginTransmission(gyro_address); // Start communication with the MPU-6050.
    Wire.write(0x6B);                     // We want to write to the PWR_MGMT_1 register (6B hex).
    Wire.write(0x00);                     // Set the register bits as 00000000 to activate the gyro.
    Wire.endTransmission();               // End the transmission with the gyro.

    Wire.beginTransmission(gyro_address); // Start communication with the MPU-6050.
    Wire.write(0x1B);                     // We want to write to the GYRO_CONFIG register (1B hex).
    Wire.write(0x08);                     // Set the register bits as 00001000 (500dps full scale).
    Wire.endTransmission();               // End the transmission with the gyro.

    Wire.beginTransmission(gyro_address); // Start communication with the MPU-6050.
    Wire.write(0x1C);                     // We want to write to the ACCEL_CONFIG register (1A hex).
    Wire.write(0x10);                     // Set the register bits as 00010000 (+/- 8g full scale range).
    Wire.endTransmission();               // End the transmission with the gyro.

    Wire.beginTransmission(gyro_address); // Start communication with the MPU-6050.
    Wire.write(0x1A);                     // We want to write to the CONFIG register (1A hex).
    Wire.write(0x03);                     // Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz).
    Wire.endTransmission();               // End the transmission with the gyro.
}

void gyro_signalen(void)
{
    Wire.beginTransmission(gyro_address); // Start communication with the gyro.
    Wire.write(0x3B);                     // Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();               // End the transmission.
    Wire.requestFrom(gyro_address, 14);
    // Request 14 bytes from the MPU 6050.
    acc_y = Wire.read() << 8 | Wire.read();       // Add the low and high byte to the acc_x variable.
    acc_x = Wire.read() << 8 | Wire.read();       // Add the low and high byte to the acc_y variable.
    acc_z = Wire.read() << 8 | Wire.read();       // Add the low and high byte to the acc_z variable.
    temperature = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the temperature variable.
    gyro_roll = Wire.read() << 8 | Wire.read();   // Read high and low part of the angular data.
    gyro_pitch = Wire.read() << 8 | Wire.read();  // Read high and low part of the angular data.
    gyro_yaw = Wire.read() << 8 | Wire.read();    // Read high and low part of the angular data.
    gyro_pitch *= -1;                             // Invert the direction of the axis.
    gyro_yaw *= -1;                               // Invert the direction of the axis.

    acc_y -= manual_acc_pitch_cal_value;       // Subtact the manual accelerometer pitch calibration value.
    acc_x -= manual_acc_roll_cal_value;        // Subtact the manual accelerometer roll calibration value.
    gyro_roll -= manual_gyro_roll_cal_value;   // Subtact the manual gyro roll calibration value.
    gyro_pitch -= manual_gyro_pitch_cal_value; // Subtact the manual gyro pitch calibration value.
    gyro_yaw -= manual_gyro_yaw_cal_value;     // Subtact the manual gyro yaw calibration value.
}

void calibrate_gyro(void)
{
    if (use_manual_calibration)
        cal_int = 2000; // If manual calibration is used set cal_int to 2000 to skip the calibration.
    else
    {
        cal_int = 0;                     // If manual calibration is not used.
        manual_gyro_pitch_cal_value = 0; // Set the manual pitch calibration variable to 0.
        manual_gyro_roll_cal_value = 0;  // Set the manual roll calibration variable to 0.
        manual_gyro_yaw_cal_value = 0;   // Set the manual yaw calibration variable to 0.
    }

    if (cal_int != 2000)
    {
        // Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
        for (cal_int = 0; cal_int < 2000; cal_int++)
        {
            gyro_signalen();              // Read the gyro output.
            gyro_roll_cal += gyro_roll;   // Add roll value to gyro_roll_cal.
            gyro_pitch_cal += gyro_pitch; // Add pitch value to gyro_pitch_cal.
            gyro_yaw_cal += gyro_yaw;     // Add yaw value to gyro_yaw_cal.
            delay(4);                     // Small delay to simulate a 250Hz loop during calibration.
        }
        gyro_roll_cal /= 2000;                        // Divide the roll total by 2000.
        gyro_pitch_cal /= 2000;                       // Divide the pitch total by 2000.
        gyro_yaw_cal /= 2000;                         // Divide the yaw total by 2000.
        manual_gyro_pitch_cal_value = gyro_pitch_cal; // Set the manual pitch calibration variable to the detected value.
        manual_gyro_roll_cal_value = gyro_roll_cal;   // Set the manual roll calibration variable to the detected value.
        manual_gyro_yaw_cal_value = gyro_yaw_cal;     // Set the manual yaw calibration variable to the detected value.
    }
}