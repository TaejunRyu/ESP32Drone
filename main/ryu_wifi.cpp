#include "ryu_wifi.h"
#include "ryu_mavlink.h"
#include "ryu_telemetry.h"

namespace WIFI
{   
esp_now_peer_info_t peer_info = {}; 

//ESP-NOW 전용
void init_wifi(){
  // NVS 초기화
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // TCP/IP 스택 초기화
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // WiFi STA 모드 초기화
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    
    // 채널 설정 (브릿지과 동일한 채널)
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    ESP_LOGI("DRONE", "WiFi STA 모드 초기화 완료, 채널: %d", ESPNOW_CHANNEL);
}

// ESP-NOW 전용 (ESP-NOW 초기화)
void init_esp_now(void) {
    // ESP-NOW 초기화
    ESP_ERROR_CHECK(esp_now_init());

    // Bridge peer 등록
    memcpy(peer_info.peer_addr, bridge_mac, ESP_NOW_ETH_ALEN);
    peer_info.channel = ESPNOW_CHANNEL;
    peer_info.encrypt = false;
    peer_info.ifidx = WIFI_IF_STA;

    esp_err_t add_peer_err = esp_now_add_peer(&peer_info);
    if (add_peer_err != ESP_OK) {
        ESP_LOGE("WIFI", "Bridge peer 등록 실패: %s", esp_err_to_name(add_peer_err));
        return;
    }         
    
    // STA(드론 방향)는 드론의 언어인 LR 모드로! (_LR은 독불장군이다. 다른 프로토콜과 같이 사용 못함.)==> (125Kbps ~ 500kbps)
    //ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR));

    {  // _LR 모드 잠시 보류하고 WIFI_PHY_RATE_2M_S 로지정함.
        ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
        // 1. 설정 구조체 준비
        esp_now_rate_config_t rate_config = {};
        rate_config.phymode = WIFI_PHY_MODE_11B;      // 802.11b 모드 (거리 확보)
        //rate_config.rate = WIFI_PHY_RATE_2M_S;         // 2Mbps Short Preamble
        rate_config.rate = WIFI_PHY_RATE_6M;            // 2Mbps Short Preamble

        
        // 해당 설정시 _LR모드와 공존할수 없다.
        esp_now_set_peer_rate_config(bridge_mac, &rate_config); 
    }
    ESP_LOGI("WIFI", "Bridge peer 등록 성공: %02x:%02x:%02x:%02x:%02x:%02x",
                 bridge_mac[0], bridge_mac[1], bridge_mac[2],
                 bridge_mac[3], bridge_mac[4], bridge_mac[5]);    
    ESP_LOGI("WIFI", "ESP-NOW 초기화 완료");
}

void connect_callback(){
    //callback 함수가 데이터를 받아서 mavlink의 데이터 처리를 하기 때문에 콜백함수를 반드시 등록해야함.
    esp_err_t err = esp_now_register_recv_cb(on_esp_now_recv);
    if (err != ESP_OK) {
        ESP_LOGE("ESP_NOW", "recv callback 등록 실패: %s", esp_err_to_name(err));
    }
    err = esp_now_register_send_cb(on_esp_now_send);
    if (err != ESP_OK) {
        ESP_LOGE("ESP_NOW", "send callback 등록 실패: %s", esp_err_to_name(err));
    }
}

int8_t current_rssi = 0;
int8_t noise_floor = 0;

/**
 * @brief 
 *      이 call back 에서 esp-now로 오는 데이터를 telemetry task로 보냄....
 * 
 * @param recv_info 
 * @param data 
 * @param len 
 */
void on_esp_now_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len <= 0 || data == nullptr || recv_info == nullptr) return;

    // 수신한 MAC 주소가 등록된 peer가 아니면 자동 등록 (양방향 통신용)
    if (!esp_now_is_peer_exist(recv_info->src_addr)) {
        esp_now_peer_info_t new_peer = {};
        memcpy(new_peer.peer_addr, recv_info->src_addr, ESP_NOW_ETH_ALEN);
        new_peer.channel = 6;
        new_peer.encrypt = false;
        new_peer.ifidx = WIFI_IF_STA;

        esp_err_t add_err = esp_now_add_peer(&new_peer);
        if (add_err == ESP_OK) {
            ESP_LOGI("ESP_NOW", "새 peer 자동 등록: %02x:%02x:%02x:%02x:%02x:%02x",
                     recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
                     recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
        } else {
            ESP_LOGW("ESP_NOW", "새 peer 등록 실패: %s", esp_err_to_name(add_err));
        }
    }
    // 실시간 정보를 구해서 bridge로 보낸다.
    current_rssi = recv_info->rx_ctrl->rssi;
    noise_floor  = recv_info->rx_ctrl->noise_floor;

    // 7,8,9번째 바이트가 메시지 ID(109)인지 확인 (Little Endian)
    // [ 00000000 ] [ data[9] ] [ data[8] ] [ data[7] ] (총 32비트 중 24비트 사용)
    uint32_t msgid =    (uint32_t)data[7]        | 
                        ((uint32_t)data[8] << 8) | 
                        ((uint32_t)data[9] << 16);

    // qgc에서 virtual joystik 운영시 packet을  어디로 보낼까?
    if (msgid == MAVLINK_MSG_ID_MANUAL_CONTROL){
        //  일단 보류 어떻게 할것인가는 추후 설정
        int16_t roll        = data[10] | (data[11] << 8);
        int16_t pitch       = data[12] | (data[13] << 8);
        int16_t throttle    = data[14] | (data[15] << 8);
        int16_t yaw         = data[16] | (data[17] << 8);
        int16_t button      = data[18] | (data[19] << 8);
        
        //*********************************************************************************************************8 */
        //printf("rc_data=> roll: %5d pitch: %5d throttle: %5d yaw: %5d button: %d\n",roll,pitch,throttle,yaw,button);

    } else {
        if (TELEM::mavlink_rx_queue != NULL) {
            TELEM::esp_now_data_t pkt;
            pkt.len = len;
            // 실제 수신된 'data'를 pkt.data로 복사해야 함!
            memcpy(pkt.data, data, len); 

            // 큐에 넣기 (ISR이 아니므로 xQueueSend 사용 가능, 대기시간 0)
            if (xQueueSend(TELEM::mavlink_rx_queue, &pkt, 0) != pdTRUE) {
                // 큐가 꽉 차서 버려지는 경우만 로그 출력
                // ESP_LOGW("ESP_NOW", "RX Queue Full"); 
            }
        }
    }
   
}

void on_esp_now_send(const wifi_tx_info_t *send_info, esp_now_send_status_t status) {
    if(status == ESP_NOW_SEND_SUCCESS){
        // 성공시 
    }else{
        // 실패시
    } 
    //ESP_LOGI("ESP_NOW", "Send status=%s", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

esp_err_t send_esp_now(const uint8_t *data, size_t len) {
    if (data == nullptr || len == 0) 
        return ESP_ERR_INVALID_ARG;
    
    // 1. ESP-NOW 최대 크기 체크 (안전 장치)
    if (len > 250) {
        ESP_LOGE("ESPNOW", "Packet too large: %d", len);
        return ESP_ERR_INVALID_SIZE;
    }

    // 2. 즉시 전송 시도
    esp_err_t err = esp_now_send(peer_info.peer_addr, data, len);

    // 3. 에러 처리 최적화
    if (err != ESP_OK) {
        if (err == ESP_ERR_ESPNOW_NO_MEM) {
            // 큐가 가득 찼을 때: 아주 잠깐 쉬고 재시도하거나 호출측에 알림
            // 파라미터 전송 루프라면 여기서 vTaskDelay(1)을 주는 것이 도움됨
            return err; 
        }
        if (err == ESP_ERR_ESPNOW_NOT_FOUND) {
            // 상대방 주소가 등록 안 됨
            return err;
        }
    }
    return err;
}


/**
 * @brief espnow로 데이터 전송시 재시도 루틴 esp_now_send()을 대신하여 재시도를 할수 있도록 처리
 * 
 * @param mac_addr 
 * @param data 
 * @param len 
 * @return esp_err_t 
 */
esp_err_t esp_now_send_retry(const uint8_t *mac_addr, const uint8_t *data, size_t len) {
    const int max_attempts = 5;
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < max_attempts; i++) {
        err = esp_now_send(mac_addr, data, len);
        if (err == ESP_OK) {
            if (i > 0) {
                ESP_LOGI("BRIDGE", "ESP-NOW 전송 성공 (재시도 %d/%d 후)", i + 1, max_attempts);
            }
            return ESP_OK;
        }
        if (err != ESP_ERR_ESPNOW_NO_MEM && err != ESP_ERR_NO_MEM) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1 + i * 1));
    }
    ESP_LOGE("BRIDGE", "ESP-NOW 전송 최종 실패: %s (최대 %d회 재시도)", esp_err_to_name(err), max_attempts);
    return err;
}



QueueHandle_t mavlink_tx_queue = NULL;
void mavlink_tx_task(void *pvParameters) {
    mav_tx_packet_t tx_pkt;
    if(mavlink_tx_queue ==NULL) return;

    while (true) {
        // 큐에서 데이터 대기 (데이터가 올 때까지 Blocked 상태로 CPU 점유 0)
        if (xQueueReceive(mavlink_tx_queue, &tx_pkt, portMAX_DELAY) == pdPASS) {
            // 2. 정상 전송
            // ESP_OK : succeed - 
            // ESP_ERR_ESPNOW_NOT_INIT : ESPNOW is not initialized - 
            // ESP_ERR_ESPNOW_ARG : invalid argument - 
            // ESP_ERR_ESPNOW_INTERNAL : internal error - 
            // ESP_ERR_ESPNOW_NO_MEM : out of memory, when this happens, you can delay a while before sending the next data - 
            // ESP_ERR_ESPNOW_NOT_FOUND : peer is not found - 
            // ESP_ERR_ESPNOW_IF : current Wi-Fi interface doesn't match that of peer - 
            // ESP_ERR_ESPNOW_CHAN: current Wi-Fi channel doesn't match that of peer
            esp_err_t result = esp_now_send(WIFI::bridge_mac, tx_pkt.buffer, tx_pkt.len);
            if(result == ESP_ERR_ESPNOW_NO_MEM){
                // 버퍼가 찰 때까지 너무 빨리 보낸 것이므로 잠시 쉬어줍니다.
                vTaskDelay(pdMS_TO_TICKS(1)); 
                // 재시도 로직 실행
                esp_now_send(WIFI::bridge_mac, tx_pkt.buffer, tx_pkt.len);
            } else if (result != ESP_OK) {
                // 그 외 에러는 기록 후 패킷 폐기 (무한 루프 방지)
                ESP_LOGD("WIFI", "Send failed: %s", esp_err_to_name(result));
            }
        }
    }
}


/**
 * @brief 모든 MAVLink 메시지 송신의 유일한 통로
 * @param msg 구성이 완료된 mavlink_message_t 구조체
 */
void dispatch_mavlink_msg(mavlink_message_t *msg) {
    if ( mavlink_tx_queue == NULL){
        return;
    }
    mav_tx_packet_t pkt;
    pkt.len = mavlink_msg_to_send_buffer(pkt.buffer, msg);

    // ISR(콜백) 여부에 따른 안전한 큐 삽입
    if (xPortInIsrContext()) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(mavlink_tx_queue, &pkt, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } else {
        xQueueSend(mavlink_tx_queue, &pkt, 0/*portMAX_DELAY */);
    }
}

}

