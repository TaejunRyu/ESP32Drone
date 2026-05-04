#pragma once

#include <esp_pm.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace Controller{

class  Flight{
    private:
        Flight();
    public:
        // 🌟 복사 생성자와 대입 연산자 비활성화 (싱글톤 복사 방지)
        Flight(const Flight&) = delete;
        Flight& operator=(const Flight&) = delete;
        ~Flight();

        // 싱글톤 인스턴스 접근 메서드
        // 🌟 get_instance() 메서드 구현
        static Flight& get_instance() {
            static Flight* instance = new Flight(); // 힙에 할당하여 소멸 순서 꼬임 방지
            return *instance;
        }  

        static inline constexpr uint8_t     ERROR_MAX_NUM   = 10; //최대 에러 발생 한계값
        static inline constexpr uint8_t     ERROR_CNT_NUM   = 3; //연속적인 에러 발생 수. (발생수가 넘으면 counting 한다) 

        static inline constexpr  uint64_t   INTERVAL_US     = 2500ULL;
        static inline constexpr  float      dt              = 0.0025f;  // 400hz

        uint8_t imu_error_cnt   =0;
        uint8_t mag_error_cnt   =0;
        uint8_t baro_error_cnt  =0;

        uint8_t imu_active_index    =0;
        uint8_t mag_active_index    =0;
        uint8_t baro_active_index   =0;

        uint64_t  total_us  =0;
        
        esp_err_t initialize();
        static void flight_task(void *pvParameters);
        void start_task();
        
    private:
        TaskHandle_t _task_handle = nullptr;
        float calculated_dt =0;
        bool _initialized = false;
        static const char* TAG;
};





















}