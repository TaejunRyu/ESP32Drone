#pragma once

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "config.hpp"
// QMC5883L 확인: 
// 시중의 GY-271/273 모듈 중 상당수가 HMC5883L이 아닌 QMC5883L 칩을 사용합니다. 
// 두 칩은 주소(0x0D)와 레지스터 구조가 달라 위 코드로 작동하지 않을 수 있으니 칩 표면 마킹을 반드시 확인하세요.
// 코드에 적힌 주석대로 모듈 칩 표면에
// "L883"이 써있으면 위 코드를 사용하시고, 
// "DA5883" 등이 써있으면 QMC5883L용 코드로 다시 작성해야 합니다.
// 작업 중인 모듈의 칩 마킹(표면 글자)을 확인해 보셨나요?

#define QMC5883L_ADDR 0x0D

void qmc5883l_init() {
    // 1. SET/RESET Period 설정 (안정적인 동작을 위해 필수)
    uint8_t set_reset[] = {0x0B, 0x01};
    i2c_master_transmit(mag_handle, set_reset, 2, pdMS_TO_TICKS(100));

    // 2. Control Register 1 설정 (0x09)
    // 설정값 설명: OSR=512, RNG=8G, ODR=100Hz, Mode=Continuous
    // 비트 계산: 00(OSR) | 01(RNG) | 10(ODR) | 01(Mode) -> 0x1D (또는 취향껏 설정)
    uint8_t config_cmd[] = {0x09, 0x1D}; 
    i2c_master_transmit(mag_handle, config_cmd, 2, pdMS_TO_TICKS(100));

    ESP_LOGI("QMC5883L", "초기화 완료 (주소: 0x0D, 연속측정모드)");
}

void read_qmc5883l(float *mx, float *my, float *mz) {
    if(mag_handle == NULL) return;

    uint8_t reg = 0x00; // X LSB부터 시작
    uint8_t data[6];

    // 6바이트 읽기 (X_L, X_H, Y_L, Y_H, Z_L, Z_H)
    if (i2c_master_transmit_receive(mag_handle, &reg, 1, data, 6, pdMS_TO_TICKS(50)) == ESP_OK) {
        // QMC5883L은 Little-Endian 방식
        int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
        int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
        int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);

        // 드론의 기체 좌표계에 맞춰 축 정렬 (필요시 수정)
        *mx = (float)raw_x;
        *my = (float)raw_y;
        *mz = (float)raw_z;
    }
}



// 지자계 하드 아이언 보정 함수 (사용자에게 8자 운동을 시키면서 최대/최소값을 수집하여 오프셋 계산)
void calibrate_mag_hard_iron() {
    float mx, my, mz;
    float mx_max = -99999.0f, mx_min = 99999.0f;
    float my_max = -99999.0f, my_min = 99999.0f;
    float mz_max = -99999.0f, mz_min = 99999.0f;

    ESP_LOGI("CALIB", "지자계 보정 시작: 드론을 모든 방향(8자)으로 돌리세요 (약 30초)...");

    // 약 3000번 샘플링 (100Hz 기준 약 30초)
    for (int i = 0; i < 3000; i++) {
        // 이전에 완성한 read_qmc5883l 함수를 호출하여 원시 데이터를 가져옵니다.
        // 주의: read_qmc5883l 내부에서 이미 OFFSET을 빼고 있다면, 
        // 보정 중에는 OFFSET이 0인 상태에서 실행하거나 raw 값을 직접 읽어야 합니다.
        read_qmc5883l(&mx, &my, &mz); 

        // 각 축의 최대/최소값 갱신
        if (mx > mx_max) mx_max = mx;
        if (mx < mx_min) mx_min = mx;
        if (my > my_max) my_max = my;
        if (my < my_min) my_min = my;
        if (mz > mz_max) mz_max = mz;
        if (mz < mz_min) mz_min = mz;

        // 100Hz 주기를 맞추기 위한 딜레이
        vTaskDelay(pdMS_TO_TICKS(10));

        // 5초마다 진행 상황 출력
        if (i % 500 == 0) {
            ESP_LOGI("CALIB", "보정 진행 중... (%d%%)", (i * 100) / 3000);
        }
    }

    // 최종 하드 아이언 오프셋(중심점) 계산
    float offset_x = (mx_max + mx_min) / 2.0f;
    float offset_y = (my_max + my_min) / 2.0f;
    float offset_z = (mz_max + mz_min) / 2.0f;

    // 센서 감도(Soft-Iron 성분 요약)를 위한 스케일 계산 (선택 사항)
    float avg_delta_x = (mx_max - mx_min) / 2.0f;
    float avg_delta_y = (my_max - my_min) / 2.0f;
    float avg_delta_z = (mz_max - mz_min) / 2.0f;
    float avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3.0f;

    float scale_x = avg_delta / avg_delta_x;
    float scale_y = avg_delta / avg_delta_y;
    float scale_z = avg_delta / avg_delta_z;

    ESP_LOGW("CALIB", "보정 완료! 아래 값을 config_ryu.hpp의 MAG_OFFSET에 적용하세요:");
    printf("#define MAG_OFFSET_X %.2f\n", offset_x);
    printf("#define MAG_OFFSET_Y %.2f\n", offset_y);
    printf("#define MAG_OFFSET_Z %.2f\n", offset_z);
    
    // 고급 보정을 위해 스케일 값도 출력
    printf("// Scale Factor (Soft-Iron): X:%.2f, Y:%.2f, Z:%.2f\n", scale_x, scale_y, scale_z);
}
