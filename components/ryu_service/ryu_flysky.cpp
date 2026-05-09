#include "ryu_flysky.h"

#include <algorithm>
#include <driver/uart.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "ryu_config.h"
#include "ryu_failsafe.h"
#include "ryu_flight_event.h"
#include "ryu_buzzer.h"
#include "ryu_utils.h"


namespace Service
{ 
 
/*
--------------시동(Armed) 걸 때의 Yaw 활용-----------------
질문하셨던 Armed(시동) 조건에서도 이 Yaw 스틱을 사용합니다.
시동 걸기: 왼쪽 스틱을 오른쪽 아래 구석으로 2초간 유지 (Throttle 최소 + Yaw 최대 오른쪽)
시동 끄기: 왼쪽 스틱을 왼쪽 아래 구석으로 2초간 유지 (Throttle 최소 + Yaw 최대 왼쪽)
*/

esp_err_t Flysky::initialize()
{
    if (_initialized) return ESP_OK;

    mcpwm_capture_timer_config_t timer_config = {};
    timer_config.group_id = 0;
    timer_config.clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;

    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&timer_config, &_cap_timer));

    mcpwm_cap_channel_handle_t cap_chan = NULL;
    mcpwm_capture_channel_config_t chan_config = {};
    chan_config.gpio_num = FLYSKY_PPM_PIN;
    chan_config.prescale = 1;
    chan_config.flags.pos_edge = false;
    chan_config.flags.neg_edge = true;
 
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(_cap_timer, &chan_config, &cap_chan));

    mcpwm_capture_event_callbacks_t cbs = {};
    cbs.on_cap = ppm_capture_callback;

    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, this));
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(_cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(_cap_timer));

    _rc_data.type = RC_NONE;

    _initialized = true;
    ESP_LOGI(TAG,"Initialized successfully.");
    return ESP_OK;
}

void Flysky::flysky_task(void *pvParameters)
{
    auto flysky = static_cast< Flysky*>(pvParameters);
    uint32_t local_ppm[MAX_CHANNELS];

    while (true) {
        uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        if (notificationValue > 0){
            portENTER_CRITICAL_SAFE(&flysky->_my_spinlock);
            memcpy(local_ppm, (const void*)flysky->_ppm_values, sizeof(local_ppm));
            portEXIT_CRITICAL_SAFE(&flysky->_my_spinlock);

            Service::rc_data_t m_rc {};
            m_rc.throttle = std::clamp((static_cast<float>(local_ppm[2]) - 1000.0f) * THR_SCALE, 0.0f, 100.0f);
            
            //롤/피치: -100 ~ 100 변환 및 Deadzone 적용
            // m_rc.roll     = Utils::Apply_DeadZone((static_cast<float>(local_ppm[0]) - 1500.0f) * ATT_SCALE, DEADZONE_RP);
            // m_rc.pitch    = Utils::Apply_DeadZone((static_cast<float>(local_ppm[1]) - 1500.0f) * ATT_SCALE, DEADZONE_RP);
            // m_rc.yaw      = Utils::Apply_DeadZone((static_cast<float>(local_ppm[3]) - 1500.0f) * ATT_SCALE, DEADZONE_YAW);

            m_rc.roll     = static_cast<float>(local_ppm[0]) - 1500.0f * ATT_SCALE;
            m_rc.pitch    = static_cast<float>(local_ppm[1]) - 1500.0f * ATT_SCALE;
            m_rc.yaw      = static_cast<float>(local_ppm[3]) - 1500.0f * ATT_SCALE;


            // 4. 스위치 처리 (간결한 삼항 연산자 구조)
            m_rc.aux1 = (local_ppm[4] > 1500) ? 1 : 0;
            m_rc.aux2 = (local_ppm[5] > 1500) ? 1 : 0;
            
            // SWC (3단 스위치)
            const uint32_t swc = local_ppm[6];
            m_rc.aux3 = (swc < 1300) ? 0 : (swc <= 1700) ? 1 : 2;
            
            m_rc.aux4 = (local_ppm[7] > 1500) ? 1 : 0;
          
            m_rc.type  = Service::RC_FLYSKY;
            portENTER_CRITICAL(&flysky->_my_spinlock);
            flysky->_rc_data = m_rc;
            portEXIT_CRITICAL(&flysky->_my_spinlock);

            if ( flysky->_rc_data.aux1 > 0){
                g_sys.manual_hold_mode =true;
            }else{
                g_sys.manual_hold_mode =false;
            }

            // 시동(Arming) 로직
            if (!g_sys.is_armed) {
                if (flysky->is_arming_gesture(m_rc)) {
                    g_sys.is_armed = true;
                    esp_event_post(Event::SYS_MODE_EVENT_BASE,Event::MODE_ARM,nullptr,0,0);   
                    Driver::Buzzer::get_instance().sound_connected();
                }
            } else {
                if (flysky->is_disarming_gesture(m_rc)) {
                    g_sys.is_armed = false;
                    esp_event_post(Event::SYS_MODE_EVENT_BASE,Event::MODE_DISARM,nullptr,0,0);   
                    Driver::Buzzer::get_instance().sound_disconnected();
                }
            }

        }else{
            // 100ms 타임아웃 발생 (신호 유실)
            if (g_sys.is_armed) {
                portENTER_CRITICAL(&flysky->_my_spinlock);
                flysky->_rc_data.throttle = 0;
                flysky->_rc_data.roll = 0;
                flysky->_rc_data.pitch = 0;
                flysky->_rc_data.yaw = 0;
                portEXIT_CRITICAL(&flysky->_my_spinlock);
                // 필요하다면 시동을 강제로 끄는 로직 추가
                // g_sys.is_armed = false;                 
    
                //ESP_LOGW(TAG, "RC LOST - Failsafe Active");

            }
        }
    }
}


bool IRAM_ATTR Flysky::ppm_capture_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data) {
    auto flysky = static_cast<Flysky*>(user_data);
    static uint32_t last_edge = 0;
    static uint32_t temp_buffer[MAX_CHANNELS]; // 임시 버퍼
    
    uint32_t current_edge = edata->cap_value;
    uint32_t pulse_width = (current_edge - last_edge) / 80;
    last_edge = current_edge;

    if (pulse_width > 3000) {
        // 모든 채널이 다 들어왔을 때만 복사 및 통지
        if (flysky->_current_channel == MAX_CHANNELS) {
            BaseType_t high_priority_task_woken = pdFALSE;
            
            portENTER_CRITICAL_SAFE(&flysky->_my_spinlock);
            memcpy(flysky->_ppm_values, temp_buffer, sizeof(temp_buffer));
            portEXIT_CRITICAL_SAFE(&flysky->_my_spinlock);

            // Flysky 태스크에 데이터 도착 알림 전송
            vTaskNotifyGiveFromISR(flysky->_task_handle, &high_priority_task_woken);
            
            if (high_priority_task_woken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }
        flysky->_current_channel = 0;
    } else {
        if (flysky->_current_channel < MAX_CHANNELS) {
            temp_buffer[flysky->_current_channel++] = pulse_width;
        }
    }
    return false;
}




bool Flysky::check_gesture(bool condition, uint32_t &counter)
{
    if (condition) [[unlikely]] { // 제스처는 비행 중 드문 케이스이므로 최적화
        if (++counter >= GESTURE_DURATION_COUNT) {
            counter = 0;
            return true;
        }
    } else {
        counter = 0;
    }
    return false;
}

bool Flysky::is_arming_gesture(const rc_data_t& rc)
{
    static uint32_t arm_cnt = 0;
    return check_gesture(_rc_data.throttle < LOW_THROTTLE_THRESHOLD && _rc_data.yaw > YAW_ARM_THRESHOLD, arm_cnt);

}

bool Flysky::is_disarming_gesture(const rc_data_t& rc) {
    static uint32_t disarm_cnt = 0;
    return check_gesture(_rc_data.throttle < LOW_THROTTLE_THRESHOLD && _rc_data.yaw < -YAW_ARM_THRESHOLD, disarm_cnt);
}

BaseType_t Flysky::start_task()
{
    auto res = xTaskCreatePinnedToCore(flysky_task,"flysky",4096,this,12,&_task_handle,0);
    if (res != pdPASS) ESP_LOGE(TAG, "❌ 1.Flysky Task is failed!  code: %d", res);
    else ESP_LOGI(TAG, "✓ 1.Flysky Task is passed...");
    return res;
}

} // namespace Service