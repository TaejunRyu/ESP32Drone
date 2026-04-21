#pragma once

#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_log.h>

namespace BATT{



// 변수 실제 정의
extern adc_oneshot_unit_handle_t adc_unit_handle;
extern adc_cali_handle_t adc_cali_handle;
extern bool do_calibration;


// 함수 선언
extern void initialize(void);
extern float get_battery_voltage(void);
extern void battery_check_task(void *pvParameters);
}