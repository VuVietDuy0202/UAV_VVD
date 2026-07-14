#pragma once
#include <Arduino.h>

// ╔══════════════════════════════════════════════════════════╗
// ║  wifi_log.h                                             ║
// ║  ESP32 Core 0 — UDP broadcast only, no HTTP server      ║
// ║  Laptop (Node.js) lo việc serve HTML + WebSocket        ║
// ╚══════════════════════════════════════════════════════════╝

// ── Cấu hình ────────────────────────────────────────────────
#define WIFI_LOG_ENABLED 0
#define WIFI_AP_SSID     "Drone_Log"
#define WIFI_AP_PASS     "12345678"
#define UDP_BROADCAST_IP "192.168.4.255"  // broadcast toàn AP subnet
#define UDP_PORT         1234
#define LOG_BUF_SIZE     64               // phải là lũy thừa 2

// ── Struct log packet ────────────────────────────────────────
struct LogPacket {
    uint32_t ts;           // micros()

    // Góc thực tế
    float pitch;           // angle_pitch    (độ)
    float roll;            // angle_roll     (độ)
    float yaw_rate;        // gyro_yaw_input (độ/s)

    // Setpoint
    float pitch_sp;        // pid_pitch_setpoint
    float roll_sp;         // pid_roll_setpoint
    float yaw_sp;          // pid_yaw_setpoint

    // PID gains (ghi nhận thông số đang dùng lúc bay)
    float kp_roll;         // pid_p_gain_roll
    float ki_roll;         // pid_i_gain_roll
    float kd_roll;         // pid_d_gain_roll
    float kp_pitch;        // pid_p_gain_pitch
    float ki_pitch;        // pid_i_gain_pitch
    float kd_pitch;        // pid_d_gain_pitch
    float kp_yaw;          // pid_p_gain_yaw
    float ki_yaw;          // pid_i_gain_yaw
    float kd_yaw;          // pid_d_gain_yaw

    // PID output
    float pid_out_pitch;   // pid_output_pitch
    float pid_out_roll;    // pid_output_roll
    float pid_out_yaw;     // pid_output_yaw

    uint8_t err;           // error code (5 = loop overrun)
};

// ── API ─────────────────────────────────────────────────────

// Gọi 1 lần trong setup() — SAU motors_init()
void wifi_log_init();

// Gọi trong loop() sau calculate_pid() — NON-BLOCKING ~10ns
void wifi_log_push(const LogPacket& pkt);

// FreeRTOS task — KHÔNG gọi trực tiếp
void wifi_log_task(void* pvParameters);