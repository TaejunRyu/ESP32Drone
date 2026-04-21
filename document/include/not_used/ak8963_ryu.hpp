#pragma once

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "config.hpp"

void init_ak8963() {
    // 1. MPU9250 Bypass 활성화
    uint8_t bypass_data[] = {0x37, 0x02};
    esp_err_t err = i2c_master_transmit(imu_handle, bypass_data, sizeof(bypass_data), pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        ESP_LOGE("AK8963", "Bypass 모드 설정 실패");
        return;
    }


    // 2. AK8963 디바이스 추가
    i2c_device_config_t mag_cfg;
    mag_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    mag_cfg.device_address  = 0x0C; // AK8963_DEVICE_ADDRESS
    mag_cfg.scl_speed_hz    = 400000;
    mag_cfg.scl_wait_us    = 0;
    mag_cfg.flags.disable_ack_check = 0; // ACK 체크 활성화    

    if (i2c_master_bus_add_device(i2c_handle, &mag_cfg, &mag_handle) != ESP_OK) {
        ESP_LOGE("AK8963", "Bus 추가 실패");
        return;
    }

    // 3. WHO_AM_I 확인 (통신 검증용)
    uint8_t who_reg = 0x00;
    uint8_t who_val = 0;
    i2c_master_transmit_receive(mag_handle, &who_reg, 1, &who_val, 1, pdMS_TO_TICKS(100));
    
    if (who_val != 0x48) {
        ESP_LOGE("AK8963", "연결 실패! ID: 0x%02X (기대값: 0x48)", who_val);
        return;
    }

    // 4. 모드 설정 (16-bit, 100Hz 연속 측정)
    uint8_t mode_cmd[] = {0x0A, 0x16};
    i2c_master_transmit(mag_handle, mode_cmd, sizeof(mode_cmd), pdMS_TO_TICKS(100));
    
    ESP_LOGI("AK8963", "초기화 성공 (ID: 0x48)");
}


void read_ak8963(float *mx, float *my, float *mz) {
    if(mag_handle == NULL) return;

    uint8_t mag_buf[7]; 
    uint8_t reg = 0x03; // HXL (X축 Low 바이트부터 시작)

    if (i2c_master_transmit_receive(mag_handle, &reg, 1, mag_buf, 7, pdMS_TO_TICKS(10)) == ESP_OK) {
        // ST2(mag_buf[6])의 HOFL(비트 3) 확인: 센서 포화 여부
        if (!(mag_buf[6] & 0x08)) {
            // Little Endian 결합 (Low | High << 8)
            int16_t raw_x = (int16_t)((mag_buf[1] << 8) | mag_buf[0]);
            int16_t raw_y = (int16_t)((mag_buf[3] << 8) | mag_buf[2]);
            int16_t raw_z = (int16_t)((mag_buf[5] << 8) | mag_buf[4]);

            // ASA 계수가 있다면 여기서 곱해줘야 함 (생략 시 오차 발생)
            // *mx = (float)raw_y * mag_asa_y; ...

            // 축 정렬 (MPU9250 표준 매핑)
            *mx = (float)raw_y;
            *my = (float)raw_x;
            *mz = (float)(-raw_z);
        }
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
        // 이전에 완성한 read_ak8963 함수를 호출하여 원시 데이터를 가져옵니다.
        // 주의: read_ak8963 내부에서 이미 OFFSET을 빼고 있다면, 
        // 보정 중에는 OFFSET이 0인 상태에서 실행하거나 raw 값을 직접 읽어야 합니다.
        read_ak8963(&mx, &my, &mz); 

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
