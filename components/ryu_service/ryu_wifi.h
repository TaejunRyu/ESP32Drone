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

namespace Service
{

class  EspNow{
    private:
        EspNow();
    public:
            // 🌟 복사 생성자와 대입 연산자 비활성화 (싱글톤 복사 방지)
        EspNow(const EspNow&) = delete;
        EspNow& operator=(const EspNow&) = delete;
        ~EspNow();

        // 싱글톤 인스턴스 접근 메서드
        // 🌟 get_instance() 메서드 구현
        static EspNow& get_instance() {
            static EspNow* instance = new EspNow(); // 힙에 할당하여 소멸 순서 꼬임 방지
            return *instance;
        }  
       

        // 통신은 channel 6번 양쪽을 바추어야한다.
        static const uint8_t   ESPNOW_CHANNEL          = 6;
        static const uint16_t  ESP_NOW_MAX_LEN         = 290;     // MAVLink v2 최대 페이로드 크기 + 헤더
        static const uint8_t   MAVLINK_TX_QUEUE_SIZE   = 60;
        static const uint8_t   MAVLINK_RX_QUEUE_SIZE   = 60;

        typedef struct {
            uint8_t buffer[290]; // MAVLink 2.0 최대 크기 대응
            uint16_t len;
        } esp_now_data_t; 
        

        int8_t current_rssi;
        int8_t noise_floor;
        
        QueueHandle_t mavlink_tx_queue;
        QueueHandle_t mavlink_rx_queue; 

        void initialize();

        static void on_esp_now_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
        static void on_esp_now_send(const wifi_tx_info_t *send_info, esp_now_send_status_t status);
        void connect_callback();
        esp_err_t send_esp_now(const uint8_t *data, size_t len);        
        void dispatch_mavlink_msg(mavlink_message_t *msg);
        std::array<uint8_t, 6> get_my_mac_address(void);
        uint16_t map_qgc_to_ibus_final(int16_t raw_val, bool is_throttle);
        static void mavlink_tx_task(void *pvParameters);
        void start_task();
        

    private:
        
        uint8_t bridge_mac[6] = {};
        esp_now_peer_info_t peer_info = {}; 
        bool _initialized;
        static const char* TAG;

};














}
