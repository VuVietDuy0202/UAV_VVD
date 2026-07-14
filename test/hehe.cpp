#include <Arduino.h>
#include <Wire.h>
#include <math.h>

constexpr uint8_t MPU_ADDRESS = 0x68;
constexpr uint32_t LOOP_US = 4000;

// Calibration values.
int16_t manual_acc_pitch_cal_value = 171;
int16_t manual_acc_roll_cal_value = -65;
int16_t manual_gyro_pitch_cal_value = 157;
int16_t manual_gyro_roll_cal_value = -168;
int16_t manual_gyro_yaw_cal_value = -101;
bool use_manual_calibration = false;

// Raw IMU data.
int16_t temperature;
int16_t acc_x, acc_y, acc_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;

// Calibration accumulators.
int16_t cal_int;
int32_t acc_total_vector;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;

// Angle values for plotting.
float angle_pitch_acc = 0.0f;
float angle_roll_acc = 0.0f;
float angle_pitch_gyro = 0.0f;
float angle_roll_gyro = 0.0f;
float angle_pitch = 0.0f;
float angle_roll = 0.0f;

uint32_t loop_timer = 0;

void gyro_setup()
{
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x1A);
    Wire.write(0x03);
    Wire.endTransmission();
}

void gyro_signalen()
{
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(MPU_ADDRESS, (uint8_t)14);

    acc_y = (int16_t)(Wire.read() << 8 | Wire.read());
    acc_x = (int16_t)(Wire.read() << 8 | Wire.read());
    acc_z = (int16_t)(Wire.read() << 8 | Wire.read());
    temperature = (int16_t)(Wire.read() << 8 | Wire.read());
    gyro_roll = (int16_t)(Wire.read() << 8 | Wire.read());
    gyro_pitch = (int16_t)(Wire.read() << 8 | Wire.read());
    gyro_yaw = (int16_t)(Wire.read() << 8 | Wire.read());

    gyro_pitch *= -1;
    gyro_yaw *= -1;

    acc_y -= manual_acc_pitch_cal_value;
    acc_x -= manual_acc_roll_cal_value;
    gyro_roll -= manual_gyro_roll_cal_value;
    gyro_pitch -= manual_gyro_pitch_cal_value;
    gyro_yaw -= manual_gyro_yaw_cal_value;
}

void calibrate_gyro()
{
    gyro_roll_cal = 0;
    gyro_pitch_cal = 0;
    gyro_yaw_cal = 0;

    if (use_manual_calibration)
    {
        cal_int = 2000;
    }
    else
    {
        cal_int = 0;
        manual_gyro_pitch_cal_value = 0;
        manual_gyro_roll_cal_value = 0;
        manual_gyro_yaw_cal_value = 0;
    }

    if (cal_int != 2000)
    {
        for (cal_int = 0; cal_int < 2000; cal_int++)
        {
            if (cal_int % 25 == 0)
            {
                digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            }

            gyro_signalen();
            gyro_roll_cal += gyro_roll;
            gyro_pitch_cal += gyro_pitch;
            gyro_yaw_cal += gyro_yaw;
            delay(4);
        }

        gyro_roll_cal /= 2000;
        gyro_pitch_cal /= 2000;
        gyro_yaw_cal /= 2000;

        manual_gyro_pitch_cal_value = gyro_pitch_cal;
        manual_gyro_roll_cal_value = gyro_roll_cal;
        manual_gyro_yaw_cal_value = gyro_yaw_cal;
    }
}

void compute_attitude()
{
    // Gyro integration at 250 Hz.
    angle_pitch_gyro += (float)gyro_pitch * 0.0000611f;
    angle_roll_gyro += (float)gyro_roll * 0.0000611f;

    // Yaw cross-coupling compensation.
    angle_pitch_gyro -= angle_roll_gyro * sin((float)gyro_yaw * 0.000001066f);
    angle_roll_gyro += angle_pitch_gyro * sin((float)gyro_yaw * 0.000001066f);

    acc_total_vector = sqrt((long)acc_x * acc_x + (long)acc_y * acc_y + (long)acc_z * acc_z);

    if (acc_total_vector > 0)
    {
        float acc_x_norm = (float)acc_x / (float)acc_total_vector;
        float acc_y_norm = (float)acc_y / (float)acc_total_vector;

        acc_x_norm = constrain(acc_x_norm, -1.0f, 1.0f);
        acc_y_norm = constrain(acc_y_norm, -1.0f, 1.0f);

        angle_pitch_acc = asin(acc_y_norm) * 57.296f;
        angle_roll_acc = asin(acc_x_norm) * 57.296f;
    }

    angle_pitch = angle_pitch * 0.9996f + angle_pitch_acc * 0.0004f;
    angle_roll = angle_roll * 0.9996f + angle_roll_acc * 0.0004f;

    static bool first_time = true;
    if (first_time)
    {
        angle_pitch = angle_pitch_acc;
        angle_roll = angle_roll_acc;
        angle_pitch_gyro = angle_pitch_acc;
        angle_roll_gyro = angle_roll_acc;
        first_time = false;
    }
}

void print_plot_data()
{
    Serial.print(F(">pitch_gyro:"));
    Serial.println(angle_pitch_gyro, 2);

    Serial.print(F(">pitch_acc:"));
    Serial.println(angle_pitch_acc, 2);

    Serial.print(F(">pitch_comp:"));
    Serial.println(angle_pitch, 2);

    Serial.print(F(">roll_gyro:"));
    Serial.println(angle_roll_gyro, 2);

    Serial.print(F(">roll_acc:"));
    Serial.println(angle_roll_acc, 2);

    Serial.print(F(">roll_comp:"));
    Serial.println(angle_roll, 2);
}

void setup()
{
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

    Wire.begin(10, 11); // SDA, SCL pins for ESP32-S3
    Wire.setClock(400000);

    gyro_setup();
    delay(500);
    calibrate_gyro();

    gyro_signalen();
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    angle_pitch_gyro = angle_pitch_acc;
    angle_roll_gyro = angle_roll_acc;

    loop_timer = micros();
}

void loop()
{
    while (micros() - loop_timer < LOOP_US)
    {
    }

    loop_timer = micros();

    gyro_signalen();
    compute_attitude();
    print_plot_data();
}