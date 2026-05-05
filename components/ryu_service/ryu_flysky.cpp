#include "ryu_flysky.h"

#include <algorithm>
#include <driver/uart.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "ryu_config.h"
#include "ryu_failsafe.h"

namespace Service
{ 
 
/*
--------------시동(Armed) 걸 때의 Yaw 활용-----------------
질문하셨던 Armed(시동) 조건에서도 이 Yaw 스틱을 사용합니다.
시동 걸기: 왼쪽 스틱을 오른쪽 아래 구석으로 2초간 유지 (Throttle 최소 + Yaw 최대 오른쪽)
시동 끄기: 왼쪽 스틱을 왼쪽 아래 구석으로 2초간 유지 (Throttle 최소 + Yaw 최대 왼쪽)
*/

const char* Flysky::TAG = "Flysky";

Flysky::Flysky(){
    ESP_LOGI(TAG,"Initializing Flysky Service...");
}
Flysky::~Flysky(){}

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

    _initialized = true;
    ESP_LOGI(TAG,"Initialized successfully.");
    return ESP_OK;
}

void Flysky::flysky_task(void *pvParameters)
{
    auto flysky = static_cast< Flysky*>(pvParameters);

    uint32_t local_ppm[MAX_CHANNELS];
    auto xLastWakeTime = xTaskGetTickCount();
    const auto xFrequency = pdMS_TO_TICKS(50); // 1 loop에 50ms  x 20번 = 1000ms = 1 second
    while (true) {
        // 1. 데이터 복사 (Critical Section 최소화)
        {
            portENTER_CRITICAL_SAFE(&flysky->_my_spinlock);
            memcpy(local_ppm, (const void*)flysky->_ppm_values, sizeof(local_ppm));
            portEXIT_CRITICAL_SAFE(&flysky->_my_spinlock);
        }

        // 2. 컴파일러 최적화 힌트 (C++23)
        // 수신기 값이 정상 범위 내에 있다고 가정하여 분기 최적화 유도
        [[assume(local_ppm[0] >= 800 && local_ppm[0] <= 2200)]];
        
        if(g_sys.is_armed){  // 신호 이상~~~~ 비상~~~~~~
            if (local_ppm[0] < 800 || local_ppm[0] > 2200){
                //auto& failsafe = Service::FailSafe::get_instance();
                //xTaskNotify(failsafe._task_handle, Service::FailSafe::ERR_RC_LOST, eSetBits);                
            }
        }

        // 1. 스로틀: 1000~2000 -> 0~100% (범위 제한 필수)
        g_rc.throttle = std::clamp((static_cast<float>(local_ppm[2]) - 1000.0f) * THR_SCALE, 0.0f, 100.0f);
        
        //롤/피치: -100 ~ 100 변환 및 Deadzone 적용
        g_rc.roll     = flysky->apply_deadzone((static_cast<float>(local_ppm[0]) - 1500.0f) * ATT_SCALE, DEADZONE_RP);
        g_rc.pitch    = flysky->apply_deadzone((static_cast<float>(local_ppm[1]) - 1500.0f) * ATT_SCALE, DEADZONE_RP);
        g_rc.yaw      = flysky->apply_deadzone((static_cast<float>(local_ppm[3]) - 1500.0f) * ATT_SCALE, DEADZONE_YAW);
        
        // 4. 스위치 처리 (간결한 삼항 연산자 구조)
        g_rc.aux1 = (local_ppm[4] > 1500) ? 1 : 0;
        g_rc.aux2 = (local_ppm[5] > 1500) ? 1 : 0;
        
        // SWC (3단 스위치)
        const uint32_t swc = local_ppm[6];
        g_rc.aux3 = (swc < 1300) ? 0 : (swc <= 1700) ? 1 : 2;
        
        g_rc.aux4 = (local_ppm[7] > 1500) ? 1 : 0;

// 잡시 테스트를 위하여 막아놈......( flight.cpp에서 시동을 걸어놔서 시동 끄는 제스처가 먹히지 않음. )
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@        
        // // 5. 시동(Arming) 로직
        // if (!g_sys.is_armed) {
        //     if (flysky->is_arming_gesture()) {
        //         g_sys.is_armed = true;
        //         BUZZ::sound_connected();
        //     }
        // } else {
        //     if (flysky->is_disarming_gesture()) {
        //         g_sys.is_armed = false;
        //         BUZZ::sound_disconnected();
        //     }
        // }
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        
        if ( g_rc.aux1 > 0){
            g_sys.manual_hold_mode =true;
        }else{
            g_sys.manual_hold_mode =false;
        }
        // 기존 printf보다 빠르며 타입 체크가 엄격합니다.
        //std::println("R:{:>5.1f} | P:{:>5.1f} | Y:{:>5.1f} | T:{:>5.1f} | Armed:{}", 
        //             flysky_roll, flysky_pitch, flysky_yaw, flysky_throttle, DRONE.is_armed);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

}

bool IRAM_ATTR Flysky::ppm_capture_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    auto flysky = static_cast< Flysky*>(user_data);

    static uint32_t last_edge = 0;
    uint32_t current_edge = edata->cap_value;
    uint32_t pulse_width = (current_edge - last_edge) / 80; // 80MHz 기준 us 단위 변환
    last_edge = current_edge;

    // PPM Sync 펄스 확인 (보통 3000us 이상이면 새로운 프레임 시작)
    if (pulse_width > 3000) {
        flysky->_current_channel = 0;
    } else {
        if (flysky->_current_channel < MAX_CHANNELS) {
            portENTER_CRITICAL_SAFE(&flysky->_my_spinlock); 
            flysky->_ppm_values[flysky->_current_channel] = pulse_width;
            flysky->_current_channel++; 
            portEXIT_CRITICAL_SAFE(&flysky->_my_spinlock);            
        }
    }
    return false;
}

float Flysky::apply_deadzone(float value, float zone)
{
    return (std::abs(value) < zone) ? 0.0f : value;
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

bool Flysky::is_arming_gesture()
{
    static uint32_t arm_cnt = 0;
    return check_gesture(g_rc.throttle < LOW_THROTTLE_THRESHOLD && 
                         g_rc.yaw > YAW_ARM_THRESHOLD, arm_cnt);

}

bool Flysky::is_disarming_gesture() {
    static uint32_t disarm_cnt = 0;
    return check_gesture(g_rc.throttle < LOW_THROTTLE_THRESHOLD && 
                         g_rc.yaw < -YAW_ARM_THRESHOLD, disarm_cnt);
}

BaseType_t Flysky::start_task()
{
    auto res = xTaskCreatePinnedToCore(flysky_task,"flysky",4096,this,12,&_task_handle,0);
    if (res != pdPASS) ESP_LOGE(TAG, "❌ 1.Flysky Task is failed!  code: %d", res);
    else ESP_LOGI(TAG, "✓ 1.Flysky Task is passed...");
    return res;
}

} // namespace Service