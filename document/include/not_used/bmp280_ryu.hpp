#pragma once

#include <math.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.hpp"

#define BMP280_ADDR         0x76
#define BMP280_REG_CALIB    0x88
#define BMP280_REG_CONTROL  0xF4
#define BMP280_REG_CONFIG   0xF5
#define BMP280_REG_DATA     0xF7

// 사용된 함수선언.
void read_calibration_data();
static float compensate_pressure(int32_t adc_P);
float get_bmp280_pressure();
void calibrate_ground_pressure();
void init_bmp280();
float get_relative_altitude();


typedef struct {
    uint16_t dig_T1; int16_t dig_T2, dig_T3;
    uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    int32_t t_fine;
} bmp280_calib_t;

static bmp280_calib_t calib;
float ground_pressure = 0.0f; 

// 1. 보정 계수 읽기 (Little Endian 조합 수정)
void read_calibration_data() {
    uint8_t reg = BMP280_REG_CALIB;
    uint8_t d[24];
    i2c_master_transmit_receive(baro_handle, &reg, 1, d, 24, -1);

    calib.dig_T1 = (uint16_t)(d[1] << 8 | d[0]);
    calib.dig_T2 = (int16_t)(d[3] << 8 | d[2]);
    calib.dig_T3 = (int16_t)(d[5] << 8 | d[4]);
    calib.dig_P1 = (uint16_t)(d[7] << 8 | d[6]);
    calib.dig_P2 = (int16_t)(d[9] << 8 | d[8]);
    calib.dig_P3 = (int16_t)(d[11] << 8 | d[10]);
    calib.dig_P4 = (int16_t)(d[13] << 8 | d[12]);
    calib.dig_P5 = (int16_t)(d[15] << 8 | d[14]);
    calib.dig_P6 = (int16_t)(d[17] << 8 | d[16]);
    calib.dig_P7 = (int16_t)(d[19] << 8 | d[18]);
    calib.dig_P8 = (int16_t)(d[21] << 8 | d[20]);
    calib.dig_P9 = (int16_t)(d[23] << 8 | d[22]);
}

// 3. 기압 보정 공식 (공식 데이터시트 준수)
static float compensate_pressure(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)calib.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;
    if (var1 == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
    return (float)p / 256.0f; // Pa를 hPa로 변환 (25600 -> 256)
}

// 2. BMP280에서 현재 기압을 읽어 보정된 값을 반환하는 함수
float get_bmp280_pressure() {
    uint8_t reg = BMP280_REG_DATA;
    uint8_t d[6];
    if (i2c_master_transmit_receive(baro_handle, &reg, 1, d, 6, pdMS_TO_TICKS(10)) != ESP_OK) return 0;

    // 온도 읽기 및 t_fine 계산 (기압 보정의 필수 전단계)
    int32_t adc_T = (int32_t)((d[3] << 12) | (d[4] << 4) | (d[5] >> 4));
    int32_t v1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
    int32_t v2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;
    calib.t_fine = v1 + v2;

    // 기압 읽기
    int32_t adc_P = (int32_t)((d[0] << 12) | (d[1] << 4) | (d[2] >> 4));
    return compensate_pressure(adc_P);
}


// 그라운드 기압 보정을 위해 여러 샘플의 평균을 계산하여 초기화하는 함수
void calibrate_ground_pressure() {
    float sum = 0;
    int count = 0;
    while(count < 100) {
        float p = get_bmp280_pressure();
        if (p > 0) {
            sum += p;
            count++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ground_pressure = sum / 100.0f;
    //ESP_LOGI("BMP280", "지면 기압 설정 완료: %.2f hPa", ground_pressure);
}

// 4. BMP280 초기화 및 설정
void init_bmp280() {
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = BMP280_ADDR;
    dev_cfg.scl_speed_hz = I2C_SPEED;
    
    
    if (i2c_master_bus_add_device(i2c_handle, &dev_cfg, &baro_handle) != ESP_OK) return;

    //read_calibration_data();
    i2c_master_transmit(baro_handle, (uint8_t[]){BMP280_REG_CONTROL, 0x57}, 2, -1);
    i2c_master_transmit(baro_handle, (uint8_t[]){BMP280_REG_CONFIG, 0x14}, 2, -1);
    

    read_calibration_data(); 
        
    // 추가: 센서가 첫 샘플을 만들 시간을 줍니다.
    vTaskDelay(pdMS_TO_TICKS(100)); 
        
    // 지면 기압 초기화 호출 (이게 실행되어야 Alt가 계산됩니다)
    calibrate_ground_pressure(); 

    ESP_LOGI("BMP280", "드라이버 초기화 완료");
}

// BMP280에서 현재 고도(지면으로부터의 상대 고도)를 계산하는 함수
float get_relative_altitude() {
    if (ground_pressure <= 0) return 0.0f;
    float p = get_bmp280_pressure();
    if (p <= 0) return 0.0f;
    float ret = 44330.0f * (1.0f - powf(p / ground_pressure, 0.1903f));
    return ret;
}
