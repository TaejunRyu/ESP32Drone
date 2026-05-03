#include "ryu_flysky.h"

#include <algorithm>
#include <driver/uart.h>
#include <driver/mcpwm_cap.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "ryu_config.h"
#include "ryu_error_proc.h"


namespace FLYSKY
{ 
    
inline constexpr uint8_t  MAX_CHANNELS  = 8;
uint32_t ppm_values[MAX_CHANNELS];
int     current_channel = 0;
portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;


// void set_ppm_values(uint32_t r,uint32_t p,uint32_t t,uint32_t y,uint32_t b){

// }



#ifdef __cplusplus
extern "C" {
#endif
static bool IRAM_ATTR ppm_capture_callback(mcpwm_cap_channel_handle_t cap_chan, 
                                            const mcpwm_capture_event_data_t *edata, 
                                            void *user_data) 
{
    static uint32_t last_edge = 0;
    uint32_t current_edge = edata->cap_value;
    uint32_t pulse_width = (current_edge - last_edge) / 80; // 80MHz 기준 us 단위 변환
    last_edge = current_edge;

    // PPM Sync 펄스 확인 (보통 3000us 이상이면 새로운 프레임 시작)
    if (pulse_width > 3000) {
        current_channel = 0;
    } else {
        if (current_channel < MAX_CHANNELS) {
            portENTER_CRITICAL_SAFE(&my_spinlock); 
            ppm_values[current_channel] = pulse_width;
            current_channel++; 
            portEXIT_CRITICAL_SAFE(&my_spinlock);            
        }
    }
    return false;
}
#ifdef __cplusplus
}
#endif

/**
 * @brief flysky 초기화 
 * 
 */
void initialize() {
    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t timer_config = {};
    timer_config.group_id = 0;
    timer_config.clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;

    mcpwm_new_capture_timer(&timer_config, &cap_timer);

    mcpwm_cap_channel_handle_t cap_chan = NULL;
    mcpwm_capture_channel_config_t chan_config = {};
    chan_config.gpio_num = FLYSKY_PPM_PIN;
    chan_config.prescale = 1;
    chan_config.flags.pos_edge = false;
    chan_config.flags.neg_edge = true;
    //chan_config.flags.pull_up = true;
    
    mcpwm_new_capture_channel(cap_timer, &chan_config, &cap_chan);

    mcpwm_capture_event_callbacks_t cbs = { .on_cap = ppm_capture_callback };
    mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, NULL);
    mcpwm_capture_channel_enable(cap_chan);
    mcpwm_capture_timer_enable(cap_timer);
    mcpwm_capture_timer_start(cap_timer);

    ESP_LOGI("FLYSKY", "✓ 드라이버 초기화 완료");
}

[[nodiscard]] constexpr float apply_deadzone(float value, float zone) noexcept {
    return (std::abs(value) < zone) ? 0.0f : value;
}

// 상수 정의 (가독성 및 유지보수성 향상)
namespace DroneConst {
    constexpr float LOW_THROTTLE_THRESHOLD = 5.0f;
    constexpr float YAW_ARM_THRESHOLD = 80.0f;
    constexpr uint32_t GESTURE_DURATION_COUNT = 100; // 2.5ms * 200 = 0.5s
}

/**
 * [[nodiscard]]: 반환값 확인 강제
 * [[gnu::always_inline]]: 함수 호출 오버헤드 제거
 */
[[nodiscard]] [[gnu::always_inline]]
inline bool check_gesture(bool condition, uint32_t& counter) noexcept {
    if (condition) [[unlikely]] { // 제스처는 비행 중 드문 케이스이므로 최적화
        if (++counter >= DroneConst::GESTURE_DURATION_COUNT) {
            counter = 0;
            return true;
        }
    } else {
        counter = 0;
    }
    return false;
}

// 시동 제스처: 스로틀 최하단 && Yaw 최우측
bool is_arming_gesture() {
    static uint32_t arm_cnt = 0;
    return check_gesture(g_rc.throttle < DroneConst::LOW_THROTTLE_THRESHOLD && 
                         g_rc.yaw > DroneConst::YAW_ARM_THRESHOLD, arm_cnt);
}

// 정지 제스처: 스로틀 최하단 && Yaw 최좌측
bool is_disarming_gesture() {
    static uint32_t disarm_cnt = 0;
    return check_gesture(g_rc.throttle < DroneConst::LOW_THROTTLE_THRESHOLD && 
                         g_rc.yaw < -DroneConst::YAW_ARM_THRESHOLD, disarm_cnt);
}



// 상수 정의 (매직 넘버 제거)

constexpr float THR_SCALE = 0.1f;    // (raw - 1000) / 10.0
constexpr float ATT_SCALE = 0.2f;    // (raw - 1500) / 5.0
constexpr float DEADZONE_RP = 2.0f;
constexpr float DEADZONE_YAW = 3.0f;
 

void flysky_task(void *pvParameters) {
    uint32_t local_ppm[MAX_CHANNELS];

    auto xLastWakeTime = xTaskGetTickCount();
    const auto xFrequency = pdMS_TO_TICKS(50); // 1 loop에 50ms  x 20번 = 1000ms = 1 second
    while (true) {
        // 1. 데이터 복사 (Critical Section 최소화)
        {
            portENTER_CRITICAL_SAFE(&my_spinlock);
            memcpy(local_ppm, (const void*)ppm_values, sizeof(local_ppm));
            portEXIT_CRITICAL_SAFE(&my_spinlock);
        }

        // 2. 컴파일러 최적화 힌트 (C++23)
        // 수신기 값이 정상 범위 내에 있다고 가정하여 분기 최적화 유도
        [[assume(local_ppm[0] >= 800 && local_ppm[0] <= 2200)]];
        
        if(g_sys.is_armed){  // 신호 이상~~~~ 비상~~~~~~
            if (local_ppm[0] < 800 || local_ppm[0] > 2200){
                xTaskNotify(ERR::xErrorHandle, ERR::ERR_RC_LOST, eSetBits);
                
            }
        }

        // 1. 스로틀: 1000~2000 -> 0~100% (범위 제한 필수)
        g_rc.throttle = std::clamp((static_cast<float>(local_ppm[2]) - 1000.0f) * THR_SCALE, 0.0f, 100.0f);
        
        //롤/피치: -100 ~ 100 변환 및 Deadzone 적용
        g_rc.roll     = apply_deadzone((static_cast<float>(local_ppm[0]) - 1500.0f) * ATT_SCALE, DEADZONE_RP);
        g_rc.pitch    = apply_deadzone((static_cast<float>(local_ppm[1]) - 1500.0f) * ATT_SCALE, DEADZONE_RP);
        g_rc.yaw      = apply_deadzone((static_cast<float>(local_ppm[3]) - 1500.0f) * ATT_SCALE, DEADZONE_YAW);
        
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
        //     if (is_arming_gesture()) {
        //         g_sys.is_armed = true;
        //         BUZZ::sound_connected();
        //     }
        // } else {
        //     if (is_disarming_gesture()) {
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

/*
--------------시동(Armed) 걸 때의 Yaw 활용-----------------
질문하셨던 Armed(시동) 조건에서도 이 Yaw 스틱을 사용합니다.
시동 걸기: 왼쪽 스틱을 오른쪽 아래 구석으로 2초간 유지 (Throttle 최소 + Yaw 최대 오른쪽)
시동 끄기: 왼쪽 스틱을 왼쪽 아래 구석으로 2초간 유지 (Throttle 최소 + Yaw 최대 왼쪽)
*/

}