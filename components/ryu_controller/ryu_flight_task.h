#pragma once

#include <esp_pm.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace Controller{

class  Flight{
    private:
        Flight() = default; 
        ~Flight() = default;
        static constexpr const char* TAG = "Flight";
    public:
        // 복사 및 이동 방지
        Flight(const Flight&) = delete;
        Flight& operator=(const Flight&) = delete;
        Flight(Flight&&) = delete;
        Flight& operator=(Flight&&) = delete;
    
        static Flight& get_instance() {
            static Flight instance; 
            return instance;
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
        BaseType_t start_task();
        void loop_check(); // loop안의 체크 목적.
        
    private:
        TaskHandle_t _task_handle = nullptr;
        float calculated_dt =0;
        bool _initialized = false;        
};





















}