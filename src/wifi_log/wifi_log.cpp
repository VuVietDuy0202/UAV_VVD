#include "wifi_log.h"
#if WIFI_LOG_ENABLED
#include <WiFi.h>
#include <WiFiUdp.h>

// ╔══════════════════════════════════════════════════════════╗
// ║  wifi_log.cpp                                           ║
// ║  ESP32 chỉ làm 1 việc: bắn UDP broadcast & quên        ║
// ║  Không có HTTP server, không có WebSocket               ║
// ║  → Node.js trên laptop lo hết phần còn lại             ║
// ╚══════════════════════════════════════════════════════════╝

// ── Latest-sample handoff (1 producer Core1 / 1 consumer Core0) ───────
// Chỉ giữ mẫu mới nhất để tránh queue bị backlog và làm loop overrun.
static LogPacket s_latest_pkt;
static volatile bool s_latest_ready = false;
static portMUX_TYPE s_pkt_mux = portMUX_INITIALIZER_UNLOCKED;

static WiFiUDP s_udp;
static IPAddress s_broadcast_ip(192, 168, 4, 255);

// ── Ring buffer API ──────────────────────────────────────────

void wifi_log_push(const LogPacket& pkt) {
    portENTER_CRITICAL(&s_pkt_mux);
    s_latest_pkt = pkt;
    s_latest_ready = true;
    portEXIT_CRITICAL(&s_pkt_mux);
}

// ── WiFi task (Core 0) ───────────────────────────────────────

void wifi_log_task(void* pvParameters) {

    // Phát WiFi AP — laptop kết nối vào đây
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    // Serial.printf("[wifi_log] AP ready  SSID: %s  IP: %s\n",
    //     WIFI_AP_SSID,
    //     WiFi.softAPIP().toString().c_str()
    // );

    IPAddress ap_ip = WiFi.softAPIP();
    s_broadcast_ip = IPAddress(ap_ip[0], ap_ip[1], ap_ip[2], 255);

    vTaskDelay(pdMS_TO_TICKS(200));

    s_udp.begin(UDP_PORT);
//    // Serial.printf("[wifi_log] UDP broadcast → %s:%d\n",
//         s_broadcast_ip.toString().c_str(), UDP_PORT
//     );

    LogPacket pkt;
    uint32_t  last_send_ms = 0;
    uint32_t  next_retry_ms = 0;
    uint32_t  last_error_log_ms = 0;

    for (;;) {
        uint32_t now = millis();

        // Gửi 50Hz và luôn dùng mẫu mới nhất để tránh backlog.
        if (now >= next_retry_ms && now - last_send_ms >= 20) {
            bool has_packet = false;
            portENTER_CRITICAL(&s_pkt_mux);
            if (s_latest_ready) {
                pkt = s_latest_pkt;
                has_packet = true;
                s_latest_ready = false;
            }
            portEXIT_CRITICAL(&s_pkt_mux);

            if (has_packet) {
                if (s_udp.beginPacket(s_broadcast_ip, UDP_PORT)) {
                    s_udp.write((const uint8_t*)&pkt, sizeof(LogPacket));
                    if (s_udp.endPacket()) {
                        last_send_ms = now;
                    } else {
                        next_retry_ms = now + 50;
                        if (now - last_error_log_ms >= 2000) {
                          //  Serial.printf("[wifi_log] UDP send failed, retrying...\n");
                            last_error_log_ms = now;
                        }
                    }
                } else {
                    next_retry_ms = now + 50;
                    if (now - last_error_log_ms >= 2000) {
                       // Serial.printf("[wifi_log] UDP beginPacket failed, retrying...\n");
                        last_error_log_ms = now;
                    }
                }
            }
        }

        // Nhường CPU — ưu tiên cho flight controller Core 1
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ── Init (gọi từ setup()) ────────────────────────────────────

void wifi_log_init() {
    xTaskCreatePinnedToCore(
        wifi_log_task,   // function
        "wifi_log",      // tên task (hiện trong debug)
        4096,            // stack — UDP không cần nhiều như WebServer
        nullptr,         // parameter
        1,               // priority thấp
        nullptr,         // handle (không cần)
        0                // Core 0
    );
}
#else
void wifi_log_push(const LogPacket& pkt) {
    (void)pkt;
}

void wifi_log_init() {}
#endif