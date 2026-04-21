#pragma once

#include <math.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.hpp"

#define BMP180_ADDR             0x77
#define BMP180_REG_CALIB        0xAA
#define BMP180_REG_CONTROL      0xF4
#define BMP180_REG_DATA         0xF6

// OSS (Over Sampling Setting): 0(low), 1(std), 2(high), 3(ultra)
#define BMP180_OSS              3 

typedef struct {
    int16_t ac1, ac2, ac3;
    uint16_t ac4, ac5, ac6;
    int16_t b1, b2, mb, mc, md;
    int32_t b5;
} bmp180_calib_t;

static bmp180_calib_t calib;
float ground_pressure = 0.0f;

// 1. 보정 계수 읽기 (Big-Endian 주의)
void read_calibration_data() {
    uint8_t reg = BMP180_REG_CALIB;
    uint8_t d[22];
    auto ret = i2c_master_transmit_receive(baro_handle, &reg, 1, d, 22, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGE("BMP180", "보정 데이터 읽기 실패");
        return;
    }

    calib.ac1 = (int16_t)(d[0] << 8 | d[1]);
    calib.ac2 = (int16_t)(d[2] << 8 | d[3]);
    calib.ac3 = (int16_t)(d[4] << 8 | d[5]);
    calib.ac4 = (uint16_t)(d[6] << 8 | d[7]);
    calib.ac5 = (uint16_t)(d[8] << 8 | d[9]);
    calib.ac6 = (uint16_t)(d[10] << 8 | d[11]);
    calib.b1  = (int16_t)(d[12] << 8 | d[13]);
    calib.b2  = (int16_t)(d[14] << 8 | d[15]);
    calib.mb  = (int16_t)(d[16] << 8 | d[17]);
    calib.mc  = (int16_t)(d[18] << 8 | d[19]);
    calib.md  = (int16_t)(d[20] << 8 | d[21]);
}

// 2. 온도 및 기압 읽기 (BMP180은 측정 명령 후 대기 필수)
float get_bmp180_pressure() {
    uint8_t reg, data[3];
    int32_t ut, up;

    // --- 온도 측정 ---
    uint8_t start_temp[] = {BMP180_REG_CONTROL, 0x2E};
    auto ret1 = i2c_master_transmit(baro_handle, start_temp, 2, pdMS_TO_TICKS(100));
    if (ret1 != ESP_OK) {
        ESP_LOGE("BMP180", "온도 측정 시작 실패");
        return 0.0f;
    }
    vTaskDelay(pdMS_TO_TICKS(5)); // 최소 4.5ms 대기

    reg = BMP180_REG_DATA;
    auto ret2 = i2c_master_transmit_receive(baro_handle, &reg, 1, data, 2, pdMS_TO_TICKS(100));
    if (ret2 != ESP_OK) {
        ESP_LOGE("BMP180", "온도 데이터 읽기 실패");
        return 0.0f;
    }
    ut = (data[0] << 8) | data[1];

    // --- 기압 측정 ---
    uint8_t start_press[] = {BMP180_REG_CONTROL, (uint8_t)(0x34 | (BMP180_OSS << 6))};
    auto ret3 = i2c_master_transmit(baro_handle, start_press, 2, pdMS_TO_TICKS(100));
    if (ret3 != ESP_OK) {
        ESP_LOGE("BMP180", "기압 측정 시작 실패");
        return 0.0f;
    }
    vTaskDelay(pdMS_TO_TICKS(26)); // OSS 3 기준 최대 25.5ms 대기

    auto ret4 = i2c_master_transmit_receive(baro_handle, &reg, 1, data, 3, pdMS_TO_TICKS(100));
    if (ret4 != ESP_OK) {
        ESP_LOGE("BMP180", "기압 데이터 읽기 실패");
        return 0.0f;
    }
    up = ((data[0] << 16) | (data[1] << 8) | data[2]) >> (8 - BMP180_OSS);

    // --- 보정 계산 (공식 데이터시트 준수) ---
    int32_t x1 = (ut - (int32_t)calib.ac6) * (int32_t)calib.ac5 >> 15;
    int32_t x2 = ((int32_t)calib.mc << 11) / (x1 + calib.md);
    calib.b5 = x1 + x2;

    int32_t b6 = calib.b5 - 4000;
    x1 = (calib.b2 * (b6 * b6 >> 12)) >> 11;
    x2 = calib.ac2 * b6 >> 11;
    int32_t x3 = x1 + x2;
    int32_t b3 = ((((int32_t)calib.ac1 * 4 + x3) << BMP180_OSS) + 2) >> 2;
    x1 = calib.ac3 * b6 >> 13;
    x2 = (calib.b1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    uint32_t b4 = (calib.ac4 * (uint32_t)(x3 + 32768)) >> 15;
    uint32_t b7 = ((uint32_t)up - b3) * (50000 >> BMP180_OSS);
    int32_t p = (b7 < 0x80000000) ? (b7 * 2 / b4) : (b7 / b4 * 2);
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4);

    return (float)p / 100.0f; // Pa -> hPa
}
void init_bmp180() {
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = BMP180_ADDR; // 0x77
    dev_cfg.scl_speed_hz = 100000; // 100kHz 고정
    
    // 장치 등록
    if (i2c_master_bus_add_device(i2c_handle, &dev_cfg, &baro_handle) != ESP_OK) {
        ESP_LOGE("BMP180", "I2C 장치 등록 실패");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(150)); 
    // [수정] 등록 후 센서가 살아있는지 ID 레지스터(0xD0) 확인
    uint8_t id_reg = 0xD0;
    uint8_t chip_id = 0;
    vTaskDelay(pdMS_TO_TICKS(100)); // 안정화 대기
    
    if (i2c_master_transmit_receive(baro_handle, &id_reg, 1, &chip_id, 1, pdMS_TO_TICKS(100)) != ESP_OK) {
        ESP_LOGE("BMP180", "센서 응답 없음 (연결 및 주소 확인 필요)");
        return;
    }
    
    if (chip_id != 0x55) {
        ESP_LOGW("BMP180", "알 수 없는 칩 ID: 0x%02X (BMP180은 0x55여야 함)", chip_id);
    }

    read_calibration_data(); 
    vTaskDelay(pdMS_TO_TICKS(100)); 
    
    // 지면 기압 보정
    float sum = 0;
    int valid_count = 0;
    for(int i=0; i<20; i++) {
        float p = get_bmp180_pressure();
        if (p > 0) {
            sum += p;
            valid_count++;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    if (valid_count > 0) {
        ground_pressure = sum / (float)valid_count;
        ESP_LOGI("BMP180", "초기화 완료. 지면기압: %.2f hPa", ground_pressure);
    } else {
        ESP_LOGE("BMP180", "기압 측정값 수집 실패");
    }
}

float get_relative_altitude() {
    if (ground_pressure <= 0) return 0.0f;
    float p = get_bmp180_pressure();
    return 44330.0f * (1.0f - powf(p / ground_pressure, 0.1903f));
}

void calibrate_ground_pressure() {
    float sum = 0;
    int count = 0;
    while(count < 100) {
        float p = get_bmp180_pressure();
        if (p > 0) {
            sum += p;
            count++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ground_pressure = sum / 100.0f;
    //ESP_LOGI("BMP180", "지면 기압 설정 완료: %.2f hPa", ground_pressure);
}


