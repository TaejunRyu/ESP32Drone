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

        static inline constexpr  uint64_t   INTERVAL_US     = 2500ULL;
        static inline constexpr  float      dt              = 0.0025f;  // 400hz

        uint64_t  total_us  =0;
        
        esp_err_t initialize();
        static void flight_task(void *pvParameters);
        BaseType_t start_task();
        void loop_check(); // loop안의 체크 목적.
        bool is_initialized(){return _initialized;};
        
    private:
        TaskHandle_t _task_handle = nullptr;
        float calculated_dt =0;
        bool _initialized = false;        
};

} //namespace Controller