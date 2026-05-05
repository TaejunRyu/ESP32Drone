#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <driver/gpio.h>
#include <esp_log.h>

namespace Driver{

class Battery{

    private:
        Battery();
    public:
            // 🌟 복사 생성자와 대입 연산자 비활성화 (싱글톤 복사 방지)
        Battery(const Battery&) = delete;
        Battery& operator=(const Battery&) = delete;
        ~Battery();

        // 싱글톤 인스턴스 접근 메서드
        // 🌟 get_instance() 메서드 구현
        static Battery& get_instance() {
            static Battery* instance = new Battery(); // 힙에 할당하여 소멸 순서 꼬임 방지
            return *instance;
        }  
        
        static inline constexpr gpio_num_t     BATTERY_ADC_PIN         = GPIO_NUM_34;
        static inline constexpr adc_channel_t  BATTERY_ADC_CH          = ADC_CHANNEL_6;
        // 저항 분배 수식 (10k / 1k 기준)
        // 현재 VOLTAGE_DIVIDER_RATIO = 11.0f (10k / 1k)를 사용 중이신데, 실제 저항의 오차(1% 등)에 따라 실제 측정값이 다를 수 있습니다.
        static inline constexpr float          VOLTAGE_DIVIDER_RATIO   = 11.0f;
        // 함수 선언
        esp_err_t initialize();
        float get_battery_voltage();
        static void battery_check_task(void *pvParameters);
        BaseType_t start_task();

    private:
        TaskHandle_t _task_handle = nullptr;
        // 변수 실제 정의
        adc_oneshot_unit_handle_t adc_unit_handle = nullptr;
        adc_cali_handle_t adc_cali_handle = nullptr;
        bool do_calibration = false;
        bool _initialized = false;
        static const char* TAG;
};




}
