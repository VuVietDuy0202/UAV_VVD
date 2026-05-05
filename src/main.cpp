#include "mpu/mpu.h"
#include "pid/pid.h"
#include "receiver/RECEIVER.h"
#include <ESP32Servo.h>

const int mot1_pin = 15;
const int mot2_pin = 16;
const int mot3_pin = 8;
const int mot4_pin = 3;
Servo mot1, mot2, mot3, mot4;
static const int ESC_HZ = 200;
int16_t esc_1, esc_2, esc_3, esc_4;
uint32_t loop_timer, error_timer;
int16_t throttle;
int16_t count_var;
boolean auto_level = true; // Auto level on (true) or off (false).

uint8_t highByte, lowByte, flip32, start;
uint8_t error, error_counter, error_led;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
float roll_level_adjust, pitch_level_adjust;

void update_attitude_from_imu()
{
  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);    // Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3); // Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);       // Gyro pid input is deg/sec.

  // 0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += (float)gyro_pitch * 0.0000611;
  angle_roll += (float)gyro_roll * 0.0000611;

  // 0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
  angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066); // If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066); // If the IMU has yawed transfer the pitch angle to the roll angel.

  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); // Calculate the total accelerometer vector.

  if (abs(acc_y) < acc_total_vector)
  {                                                                   // Prevent the asin function to produce a NaN.
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296; // Calculate the pitch angle.
  }
  if (abs(acc_x) < acc_total_vector)
  {                                                                  // Prevent the asin function to produce a NaN.
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296; // Calculate the roll angle.
  }

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004; // Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;    // Correct the drift of the gyro roll angle with the accelerometer roll angle.
}

void update_level_adjust()
{
  pitch_level_adjust = angle_pitch * 15; // Calculate the pitch angle correction.
  roll_level_adjust = angle_roll * 15;   // Calculate the roll angle correction.

  if (!auto_level)
  {                         // If the quadcopter is not in auto-level mode
    pitch_level_adjust = 0; // Set the pitch angle correction to zero.
    roll_level_adjust = 0;  // Set the roll angle correcion to zero.
  }
}

void motors_init()
{
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Set tần số TRƯỚC khi attach
  mot1.setPeriodHertz(ESC_HZ);
  mot2.setPeriodHertz(ESC_HZ);
  mot3.setPeriodHertz(ESC_HZ);
  mot4.setPeriodHertz(ESC_HZ);

  // Attach sau
  mot1.attach(mot1_pin, 1000, 2000);
  mot2.attach(mot2_pin, 1000, 2000);
  mot3.attach(mot3_pin, 1000, 2000);
  mot4.attach(mot4_pin, 1000, 2000);

  mot1.writeMicroseconds(1000);
  mot2.writeMicroseconds(1000);
  mot3.writeMicroseconds(1000);
  mot4.writeMicroseconds(1000);
  delay(2000); // ESC cần ít nhất 2-3s để nhận tín hiệu arm
}
void setup()
{
  Serial.begin(57600);
  rx_config();
  Serial.println("Starting setup...");
  Wire.setClock(400000);
  Wire.begin(11, 10);                   // Start the I2C as master
  Wire.beginTransmission(gyro_address); // Start communication with the MPU-6050.
  error = Wire.endTransmission();       // End the transmission and register the exit status.
  while (error != 0)
  {            // Stay in this loop because the MPU-6050 did not responde.
    error = 2; // Set the error status to 2.
               // error_signal(); // Show the error via the red LED.
    delay(4);
  }
  gyro_setup(); // Initiallize the gyro and set the correct registers.
  Serial.println("Gyro setup complete");
  if (!use_manual_calibration)
  {
    for (count_var = 0; count_var < 1250; count_var++)
    { // 1250 loops of 4 microseconds = 5 seconds
      if (count_var % 125 == 0)
      {                                   // Every 125 loops (500ms).
        digitalWrite(4, !digitalRead(4)); // Change the led status.
      }
      delay(4); // Delay 4 microseconds
    }
    count_var = 0; // Set start back to 0.
  }

  calibrate_gyro(); // Calibrate the gyro offset.
  Serial.println("Gyro calibrated");

  while (ReceiverValue[0] < 990 || ReceiverValue[1] < 990 || ReceiverValue[2] < 990 || ReceiverValue[3] < 990)
  {
    error = 3; // Set the error status to 3.
    delay(4);
  }
  error = 0; // Reset the error status to 0.
  while (ReceiverValue[2] < 990 || ReceiverValue[2] > 1050)
  {
    error = 4; // Set the error status to 4.
    delay(4);
  }
  error = 0; // Reset the error status to 0.
  // ====================== PWM Timer riêng cho 4 ESC ======================
  motors_init();
  loop_timer = micros(); // Set the timer for the first loop.
  Serial.println("Setup complete");
}

void loop()
{
  Serial.print("ERROR: ");
  Serial.println(error); // Show the errors via the red LED.
  gyro_signalen();
  update_attitude_from_imu();
  update_level_adjust();

  // For starting the motors: throttle low and yaw left (step 1).
  if (ReceiverValue[2] < 1050 && ReceiverValue[3] < 1300)
    start = 1;
  // When yaw stick is back in the center position start the motors (step 2).
  if (start == 1 && ReceiverValue[2] < 1050 && ReceiverValue[3] > 1450)
  {
    start = 2;
    angle_pitch = angle_pitch_acc; // Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;   // Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.

    // Reset the PID controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  // Stopping the motors: throttle low and yaw right.
  if (start == 2 && ReceiverValue[2] < 1050 && ReceiverValue[3] > 1700)
  {
    start = 0;
    // green_led(HIGH); // Turn on the green led.
  }

  pid_roll_setpoint = 0;
  if (ReceiverValue[0] > 1508)
    pid_roll_setpoint = ReceiverValue[0] - 1508;
  else if (ReceiverValue[0] < 1492)
    pid_roll_setpoint = ReceiverValue[0] - 1492;

  pid_roll_setpoint -= roll_level_adjust;
  pid_roll_setpoint /= 3.0;

  pid_pitch_setpoint = 0;
  if (ReceiverValue[1] > 1508)
    pid_pitch_setpoint = -(ReceiverValue[1] - 1508);
  else if (ReceiverValue[1] < 1492)
    pid_pitch_setpoint = -(ReceiverValue[1] - 1492);

  pid_pitch_setpoint -= pitch_level_adjust;
  pid_pitch_setpoint /= 3.0;

  pid_yaw_setpoint = 0;

  if (ReceiverValue[2] > 1050)
  { // Do not yaw when turning off the motors.
    if (ReceiverValue[3] > 1508)
      pid_yaw_setpoint = (ReceiverValue[3] - 1508) / 3.0;
    else if (ReceiverValue[3] < 1492)
      pid_yaw_setpoint = (ReceiverValue[3] - 1492) / 3.0;
  }

  calculate_pid(); // PID inputs are known. So we can calculate the pid output.
  // battery_voltage = battery_voltage * 0.92 + ((float)analogRead(4) / 1410.1);
  throttle = ReceiverValue[2]; // We need the throttle signal as a base signal.

  if (start == 2)
  { // The motors are started.
    if (throttle > 1800)
      throttle = 1800;                                                      // We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; // Calculate the pulse for esc 1 (front-right - CCW).
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; // Calculate the pulse for esc 2 (rear-right - CW).
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; // Calculate the pulse for esc 3 (rear-left - CCW).
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; // Calculate the pulse for esc 4 (front-left - CW).
    if (esc_1 < 1100)
      esc_1 = 1100; // Keep the motors running.
    if (esc_2 < 1100)
      esc_2 = 1100; // Keep the motors running.
    if (esc_3 < 1100)
      esc_3 = 1100; // Keep the motors running.
    if (esc_4 < 1100)
      esc_4 = 1100; // Keep the motors running.

    if (esc_1 > 2000)
      esc_1 = 2000; // Limit the esc-1 pulse to 2000us.
    if (esc_2 > 2000)
      esc_2 = 2000; // Limit the esc-2 pulse to 2000us.
    if (esc_3 > 2000)
      esc_3 = 2000; // Limit the esc-3 pulse to 2000us.
    if (esc_4 > 2000)
      esc_4 = 2000; // Limit the esc-4 pulse to 2000us.
  }

  else
  {
    esc_1 = 1000; // If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000; // If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000; // If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000; // If start is not 2 keep a 1000us pulse for ess-4.
  }
  mot1.writeMicroseconds(esc_1);
  mot2.writeMicroseconds(esc_2);
  mot3.writeMicroseconds(esc_3);
  mot4.writeMicroseconds(esc_4);

  if (micros() - loop_timer > 4050)
    error = 5; // Turn on the LED if the loop time exceeds 4050us.
  while (micros() - loop_timer < 4000)
    ;                    // We wait until 4000us are passed.
  loop_timer = micros(); // Set the timer for the next loop.
}
