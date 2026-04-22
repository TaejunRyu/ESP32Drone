#pragma once

#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <driver/gpio.h>
#include <esp_log.h>

namespace BATT{

inline constexpr gpio_num_t    BATTERY_ADC_PIN   = GPIO_NUM_34;
inline constexpr adc_channel_t BATTERY_ADC_CH    = ADC_CHANNEL_6;

// 변수 실제 정의
extern adc_oneshot_unit_handle_t adc_unit_handle;
extern adc_cali_handle_t adc_cali_handle;
extern bool do_calibration;


// 함수 선언
extern void initialize(void);
extern float get_battery_voltage(void);
extern void battery_check_task(void *pvParameters);
}