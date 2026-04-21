#pragma once

#include <math.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.hpp"

#define MS5611_ADDR             0x77  // CSB가 GND면 0x77, VCC면 0x76
#define MS5611_CMD_RESET        0x1E
#define MS5611_CMD_ADC_READ     0x00
#define MS5611_CMD_CONV_D1_4096 0x48  // 기압 측정 (OSR 4096)
#define MS5611_CMD_CONV_D2_4096 0x58  // 온도 측정 (OSR 4096)
#define MS5611_CMD_PROM_READ    0xA0  // PROM 읽기 시작 주소

typedef struct {
    uint16_t c1, c2, c3, c4, c5, c6; // 보정 계수
} ms5611_calib_t;

static ms5611_calib_t ms_cal;
//float ground_pressure;

// 1. PROM에서 보정 계수 읽기
void read_ms5611_calibration(i2c_master_dev_handle_t handle) {
    uint8_t rx[2];
    uint16_t prom[8];

    for (int i = 0; i < 8; i++) {
        uint8_t cmd = MS5611_CMD_PROM_READ + (i * 2);
        if (i2c_master_transmit_receive(handle, &cmd, 1, rx, 2, pdMS_TO_TICKS(100)) == ESP_OK) {
            prom[i] = (rx[0] << 8) | rx[1];
        }
    }
    // C1~C6 할당 (PROM 1~6번 데이터)
    ms_cal.c1 = prom[1]; ms_cal.c2 = prom[2];
    ms_cal.c3 = prom[3]; ms_cal.c4 = prom[4];
    ms_cal.c5 = prom[5]; ms_cal.c6 = prom[6];
    ESP_LOGI("MS5611", "✓ 보정 계수 읽기 완료: C1=%u, C6=%u", ms_cal.c1, ms_cal.c6);
}

// 2. ADC 값 읽기 (24비트)
uint32_t read_ms5611_adc(i2c_master_dev_handle_t handle, uint8_t cmd) {
    uint8_t rx[3];
    i2c_master_transmit(handle, &cmd, 1, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(10)); // OSR 4096 대기 시간 (약 9ms 필요)

    uint8_t read_cmd = MS5611_CMD_ADC_READ;
    if (i2c_master_transmit_receive(handle, &read_cmd, 1, rx, 3, pdMS_TO_TICKS(100)) == ESP_OK) {
        return (uint32_t)((rx[0] << 16) | (rx[1] << 8) | rx[2]);
    }
    return 0;
}

// 3. 실제 기압 계산 (데이터시트 공식)
float get_ms5611_pressure(i2c_master_dev_handle_t handle) {
    uint32_t d1 = read_ms5611_adc(handle,MS5611_CMD_CONV_D1_4096); // 기압 ADC
    uint32_t d2 = read_ms5611_adc(handle,MS5611_CMD_CONV_D2_4096); // 온도 ADC

    if (d1 == 0 || d2 == 0) return 0.0f;

    // 온도 계산
    int32_t dt = d2 - ((int32_t)ms_cal.c5 << 8);
    int32_t temp = 2000 + ((int64_t)dt * ms_cal.c6 >> 23);

    // 기압 계산 오프셋 및 감도
    int64_t off = ((int64_t)ms_cal.c2 << 16) + (((int64_t)ms_cal.c4 * dt) >> 7);
    int64_t sens = ((int64_t)ms_cal.c1 << 15) + (((int64_t)ms_cal.c3 * dt) >> 8);

    // 2차 보정 (온도가 20도 미만일 경우 처리)
    if (temp < 2000) {
        int64_t t2 = ((int64_t)dt * dt) >> 31;
        int64_t off2 = 5 * ((int64_t)(temp - 2000) * (temp - 2000)) >> 1;
        int64_t sens2 = 5 * ((int64_t)(temp - 2000) * (temp - 2000)) >> 2;
        temp -= t2;
        off -= off2;
        sens -= sens2;
    }

    int32_t p = (((d1 * sens) >> 21) - off) >> 15;
    return (float)p / 100.0f; // hPa 변환
}

i2c_master_dev_handle_t init_ms5611(i2c_master_bus_handle_t bus_handle) {
    i2c_master_dev_handle_t handle =NULL;

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = MS5611_ADDR;
    dev_cfg.scl_speed_hz = 100000;

    if (i2c_master_bus_add_device(bus_handle, &dev_cfg, &handle) != ESP_OK) {
        ESP_LOGE("MS5611", "I2C 장치 등록 실패");
        return NULL;
    }

    // 센서 리셋
    uint8_t reset_cmd = MS5611_CMD_RESET;
    i2c_master_transmit(handle, &reset_cmd, 1, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(50));

    read_ms5611_calibration(handle);
    vTaskDelay(pdMS_TO_TICKS(100));

    // 지면 기압 초기화 (10번 평균)
    float sum = 0;
    int count = 0;
    for(int i=0; i<10; i++) {
        float p = get_ms5611_pressure(handle);
        if (p > 500.0f) { // 유효한 값만
            sum += p;
            count++;
        }
    }
    if (count > 0) g_baro.ground_pressure = sum / count;    
    ESP_LOGI("MS5611", "✓ 초기화 완료. 지면기압: %.2f hPa", g_baro.ground_pressure);
    return handle;
}




float get_relative_altitude(i2c_master_dev_handle_t handle, float ground_pressure) {
    static float last_altitude = 0.0f; // 이전 고도 저장용
    if (ground_pressure <= 500.0f) return 0.0f; // 비정상적인 지면 기압 차단

    float p = get_ms5611_pressure(handle);
    if (p <= 500.0f) return last_altitude; // 일시적 오류 시 이전 값 유지

    // 고도 계산 공식 (ISA 모델)
    float current_alt = 44330.0f * (1.0f - powf(p / ground_pressure, 0.190295f));

    // 간단한 1차 저주파 필터 (Alpha: 0.1 ~ 0.3 권장)
    // 노이즈를 줄이고 부드러운 고도 변화를 만듭니다.
    const float alpha = 0.2f; 
    float filtered_alt = (current_alt * alpha) + (last_altitude * (1.0f - alpha));
    
    last_altitude = filtered_alt;
    return filtered_alt;
}


/**
 * @brief Get the filtered altitude object
 *      raw_alt를 받아 단순저역 통과필터(lpf)를 통해 노이즈를 잡는다.
 * @return float 
 */
float get_ms5611_filtered_altitude(i2c_master_dev_handle_t handle ,float ground_pressure) {
    static float filtered_altitude = 0.0f;
    float raw_alt = get_relative_altitude( handle,ground_pressure);
    // 단순 저역 통과 필터 (LPF): 이전 값 90%, 새 값 10% 반영
    // 드론의 급격한 고도 변화 노이즈를 잡아줍니다.
    filtered_altitude = (filtered_altitude * 0.9f) + (raw_alt * 0.1f);    
    return filtered_altitude;
}

float update_ms5611_climb_rate(float current_alt) {
    static float last_alt = 0.0f;
    static float f_climb_rate = 0.0f;

    float raw_rate = (current_alt - last_alt) / 0.025f; // 40Hz = 0.025s
    last_alt = current_alt;

    // 속도 필터 (기압계 노이즈 제거용)
    f_climb_rate = (f_climb_rate * 0.8f) + (raw_rate * 0.2f);
    return f_climb_rate;
}

// 2. 이동 평균 필터 (Moving Average Filter)
// 최근 N개의 데이터를 모아 평균을 내는 방식입니다. 정지 상태의 고도 안정성이 매우 뛰어납니다.

#define AVG_WINDOW_SIZE 10
float alt_buffer[AVG_WINDOW_SIZE] = {0};
int buf_idx = 0;

float get_moving_avg_altitude(i2c_master_dev_handle_t handle) {
    alt_buffer[buf_idx] = get_relative_altitude(handle,g_baro.ground_pressure);
    buf_idx = (buf_idx + 1) % AVG_WINDOW_SIZE;

    float sum = 0;
    for(int i = 0; i < AVG_WINDOW_SIZE; i++) {
        sum += alt_buffer[i];
    }
    return sum / AVG_WINDOW_SIZE;
}


// 시스템 시작시에 한번 실행해 현재 위치의 기압을 저장한다.
float calibrate_ms5611_ground_pressure(i2c_master_dev_handle_t handle) {
    float sum = 0;
    int count = 0;
    int attempts = 0; // 무한 루프 방지용

    ESP_LOGI("M5611", "✓ 지면 기압 보정 시작 (100회 샘플링)...");

    // 1. 센서 안정화를 위해 첫 데이터는 읽고 버림
    get_ms5611_pressure(handle);
    vTaskDelay(pdMS_TO_TICKS(50));

    while(count < 100 && attempts < 200) { // 최대 200번 시도
        float p = get_ms5611_pressure(handle);
        
        if (p > 800.0f && p < 1200.0f) { // 좀 더 타이트한 유효 범위 (지상 기준)
            sum += p;
            count++;
        } else {
            ESP_LOGW("M5611", "잘못된 압력 값 감지: %.2f hPa", p);
        }

        attempts++;
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz 샘플링

        if (count % 25 == 0 && count > 0) {
            ESP_LOGD("M5611", "보정 진행률: %d%%", count);
        }
    }

    if (count >= 50) { // 최소 50개 이상의 유효 샘플 확보 시
        float ground_p = sum / (float)count;
        ESP_LOGI("M5611", "✓ 지면 기압 설정 완료: %.2f hPa (샘플: %d개)", ground_p, count);
        return ground_p;
    }

    ESP_LOGE("M5611", "❌ 지면 기압 보정 실패 (센서 확인 필요)");
    return 0.0f;
}
