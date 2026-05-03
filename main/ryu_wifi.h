#pragma once
#include <esp_event.h>
#include <nvs_flash.h>
#include <esp_log.h>
#include <lwip/sockets.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h> 
#include <c_library_v2/common/mavlink.h>
#include <esp_wifi.h>

#include "ryu_config.h"

namespace WIFI
{

inline constexpr uint16_t  ESP_NOW_MAX_LEN   = 290;     // MAVLink v2 최대 페이로드 크기 + 헤더
inline constexpr uint8_t  MAVLINK_TX_QUEUE_SIZE  = 60;
inline constexpr uint8_t  MAVLINK_RX_QUEUE_SIZE  = 60;


    // 
// 전송용 구조체
typedef struct {
    uint8_t buffer[ESP_NOW_MAX_LEN]; // MAVLink 2.0 최대 크기 대응
    uint16_t len;
} mav_tx_packet_t;

// bridge mac_address ( 칩이 바뀌면 mac address도 바뀌므로 반드시 체크.)
// ESP32
//inline constexpr uint8_t bridge_mac[] = {0xB0, 0xCB, 0xD8, 0xD6, 0xB0, 0xC8};

//ESP32S3
inline constexpr uint8_t bridge_mac[] = {0x1C, 0xDB, 0xD4, 0xAE, 0x82, 0x04};

// 통신은 channel 6번 양쪽을 바추어야한다.
inline constexpr uint8_t ESPNOW_CHANNEL  = 6;
// MAC ADDRESS을 가진 정보
extern esp_now_peer_info_t peer_info;

// 현재 통신상의 rssi상태 
extern int8_t current_rssi;
extern int8_t noise_floor;

extern QueueHandle_t mavlink_tx_queue;


// esp-now 만 실행 시킬수 있는 함수들...
extern void init_wifi();
extern void init_esp_now();
extern void connect_callback();
extern esp_err_t send_esp_now(const uint8_t *data, size_t len);

extern void mavlink_tx_task(void *pvParameters);

extern void dispatch_mavlink_msg(mavlink_message_t *msg);

extern std::array<uint8_t, 6> get_my_mac_address(void);

extern void on_esp_now_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
extern void on_esp_now_send(const wifi_tx_info_t *send_info, esp_now_send_status_t status);

}
