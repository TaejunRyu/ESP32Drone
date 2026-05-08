#pragma once

#include <driver/gpio.h>
#include <driver/uart.h>
#include <driver/mcpwm_cap.h>
// PPM 신호 캡처 콜백 함수
/*
ppm_values[0]: Aileron (Roll - 좌우 기울기)
ppm_values[1]: Elevator (Pitch - 앞뒤 기울기)
ppm_values[2]: Throttle (스로틀 - 상승/하강)
ppm_values[3]: Rudder (Yaw - 좌우 회전)
ppm_values[4]: Aux 1 (스위치 또는 다이얼) ----> AUTO MODE
ppm_values[5]: Aux 2 (스위치 또는 다이얼)
*/

namespace Service
{

class Flysky{
    private:
        Flysky() = default; 
        ~Flysky() = default;
        static constexpr const char* TAG = "Flysky";
    public:
        static Flysky& get_instance() {
            static Flysky instance; 
            return instance;
        }
        Flysky(const Flysky&) = delete;
        Flysky& operator=(const Flysky&) = delete;
        Flysky(Flysky&&) = delete;
        Flysky& operator=(Flysky&&) = delete;
        // PPM으로 데이터를 보낼때
        static inline constexpr gpio_num_t FLYSKY_PPM_PIN = GPIO_NUM_4; 
        static inline constexpr uint8_t    MAX_CHANNELS  = 8;

        static inline constexpr float     LOW_THROTTLE_THRESHOLD = 5.0f;
        static inline constexpr float     YAW_ARM_THRESHOLD = 80.0f;
        static inline constexpr uint32_t  GESTURE_DURATION_COUNT = 100; // 2.5ms * 200 = 0.5s
        
        static inline constexpr float     THR_SCALE = 0.1f;    // (raw - 1000) / 10.0
        static inline constexpr float     ATT_SCALE = 0.2f;    // (raw - 1500) / 5.0
        static inline constexpr float     DEADZONE_RP = 2.0f;
        static inline constexpr float     DEADZONE_YAW = 3.0f;

        struct rc_data_t {
            float throttle;  // 스로틀 (0~100%)
            float roll;      // 롤 (-100~100)
            float pitch;     // 피치 (-100~100)
            float yaw;       // 요 (-100~100)
            float aux1;      // 보조 채널 1 (고도 유지)
            float aux2;      // 보조 채널 2
            float aux3;      // 보조 채널 3 (SWC 3단)
            float aux4;      // 보조 채널 4
        };
                
        esp_err_t initialize();
        bool is_initialized(){return _initialized;};
        static void flysky_task(void *pvParameters);
        static bool ppm_capture_callback( mcpwm_cap_channel_handle_t cap_chan, 
                                             const mcpwm_capture_event_data_t *edata, 
                                             void *user_data);
        // 외부(PID Task)에서 데이터를 안전하게 읽어갈 때 사용
        void get_latest_rc(rc_data_t* out_data) {
            portENTER_CRITICAL(&_my_spinlock);
            *out_data = _rc_data; // 구조체 복사
            portEXIT_CRITICAL(&_my_spinlock);
        }
        bool check_gesture(bool condition, uint32_t& counter);
        bool is_arming_gesture(const rc_data_t& rc);
        bool is_disarming_gesture(const rc_data_t& rc);
        portMUX_TYPE* get_spinlock(){return &_my_spinlock;};
        BaseType_t start_task();

    private:
        rc_data_t _rc_data = {}; // 내부 저장용

        TaskHandle_t    _task_handle = nullptr;
        int             _current_channel = 0;
        uint32_t        _ppm_values[MAX_CHANNELS]={0,};        
        portMUX_TYPE    _my_spinlock = portMUX_INITIALIZER_UNLOCKED;

        mcpwm_cap_timer_handle_t _cap_timer = nullptr;
        bool _initialized = false;
}; 

 
}// 