#include "ryu_battery.h"

#include "ryu_config.h"
#include "ryu_error_proc.h"

/**
 * @brief 팁: 멀티미터로 실제 배터리 전압을 재보고, 
 *            get_battery_voltage() 출력값과 차이가 있다면 이 값을 소수점 단위로 미세 조정(예: 11.12f)하여 캘리브레이션하세요.
 * 
 */
namespace BATT
{
// 저항 분배 수식 (10k / 1k 기준)
// 현재 VOLTAGE_DIVIDER_RATIO = 11.0f (10k / 1k)를 사용 중이신데, 실제 저항의 오차(1% 등)에 따라 실제 측정값이 다를 수 있습니다.
inline constexpr float VOLTAGE_DIVIDER_RATIO  = 11.0f;

adc_oneshot_unit_handle_t adc_unit_handle = NULL;
adc_cali_handle_t adc_cali_handle = NULL;
bool do_calibration = false;

void initialize(void) {
    // 1. ADC 유닛 초기화
    adc_oneshot_unit_init_cfg_t init_config = {};
    init_config.unit_id = ADC_UNIT_1;
    
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_unit_handle));

    // 2. ADC 채널 설정 (GPIO 34 = ADC_CHANNEL_6)
    adc_oneshot_chan_cfg_t config = {
        .atten =  adc_atten_t::ADC_ATTEN_DB_12, // 12dB 감쇄 (v5.0 명칭)
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_unit_handle, BATTERY_ADC_CH, &config));

    // 3. ADC 보정 (에러 메시지 제안: line_fitting 사용)
    adc_cali_line_fitting_config_t cali_config = {};
    cali_config.unit_id = ADC_UNIT_1;
    cali_config.atten = ADC_ATTEN_DB_12;
    cali_config.bitwidth = ADC_BITWIDTH_DEFAULT;
    
    // 함수명 정정: adc_cali_create_scheme_line_fitting
    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_config, &adc_cali_handle);
    if (ret == ESP_OK) {
        do_calibration = true;
    }
}

float get_battery_voltage(void) {
    int adc_raw = 0;
    int voltage_mv = 0;

    // 1. Raw 값 읽기
    adc_oneshot_read(adc_unit_handle, BATTERY_ADC_CH, &adc_raw);

    // [추가] Zero-cut 로직: Raw가 0이면 보정 수식 무시하고 0V 반환
    if (adc_raw == 0) {
        return 0.0f;
    }
    // 2. 보정 적용 (Raw가 0이 아닐 때만 실행)
    if (do_calibration && adc_cali_handle != NULL) {
        adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &voltage_mv);
    } else {
        voltage_mv = (adc_raw * 3113) / 4095; // 3.3V 연결 시 확인한 3113mV 기준
    }
    //printf("DEBUG: Raw=%d, mV=%d\n", adc_raw, voltage_mv);
    // 3. 실제 전압 복원 (11.0배)
    return (voltage_mv / 1000.0f) * VOLTAGE_DIVIDER_RATIO;
}

// 예시: 10번 읽어서 평균 내기
float get_avg_battery_voltage(void) {
    int adc_raw = 0;
    int voltage_mv = 0;
    int adc_raw_sum = 0;
    const int samples = 10;
    
    for (int i = 0; i < samples; i++) {
        int temp_raw = 0;
        adc_oneshot_read(adc_unit_handle, BATTERY_ADC_CH, &temp_raw);
        adc_raw_sum += temp_raw;
    }
    adc_raw = adc_raw_sum / samples;
    
    if (adc_raw == 0) return 0.0f;
      // 2. 보정 적용 (Raw가 0이 아닐 때만 실행)
    if (do_calibration && adc_cali_handle != NULL) {
        adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &voltage_mv);
    } else {
        voltage_mv = (adc_raw * 3113) / 4095; // 3.3V 연결 시 확인한 3113mV 기준
    }
    //printf("DEBUG: Raw=%d, mV=%d\n", adc_raw, voltage_mv);
    // 3. 실제 전압 복원 (11.0배)
    return (voltage_mv / 1000.0f) * VOLTAGE_DIVIDER_RATIO;
}

/**
 * @brief 주의사항: get_battery_voltage()에서 0V가 나올 때는 배터리가 없는 상태일 수 있으니, 
 *        voltage > 0.5f 처럼 배터리 연결 여부를 먼저 확인하는 조건문을 넣는 것이 좋습니다.
 * 
 * @param pvParameters 
 */
void battery_check_task(void* pvParameters) {
    while (true) {
        float voltage = get_battery_voltage();
        // 예: 1셀당 3.5V 미만일 때 (3S 배터리 기준 10.5V)
        if (voltage > 0.5f && voltage < 10.5f) { 
            // 에러 핸들러 태스크에 '저전압 비트' 세팅
            xTaskNotify(ERR::xErrorHandle, ERR::ERR_BATTERY_LOW, eSetBits);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1초마다 확인
    }
}

}