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

#define HMC5883L_ADDR               0x1E    // 센서 주소

// 자신의 센서에 맞는 오프셋 값으로 수정 필요
//calibrate_mag_hard_iron() 함수를 통해 얻은 오프셋 값을 여기에 입력하세요.
static constexpr float MAG_HMC5883_OFFSET_X = 251; 
static constexpr float MAG_HMC5883_OFFSET_Y = 245;
static constexpr float MAG_HMC5883_OFFSET_Z = 2;

// 2. HMC5883L 초기 설정 (연속 측정 모드)
// --- 초기화 부분 ---
i2c_master_dev_handle_t init_hmc5883l(i2c_master_bus_handle_t bus_handle) {
    i2c_master_dev_handle_t handle = nullptr;

    // 1. 장치 등록을 먼저 수행
    i2c_device_config_t device_cfg = {};
    device_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7; 
    device_cfg.device_address = HMC5883L_ADDR; // 0x1E
    device_cfg.scl_speed_hz = 200000; // HMC5883L은 100kHz ~ 200kHz 권장 (기존 코드에서는 100kHz로 설정되어 있었음)

    auto ret = i2c_master_bus_add_device(bus_handle, &device_cfg, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE("HMC5883L", "I2C 장치 등록 실패 (연결 확인 필요)");
        return nullptr;
    } 
    
    vTaskDelay(pdMS_TO_TICKS(20)); 

    // 3. 실제 HMC5883L 설정값 기록 (중요: 이 과정이 없으면 0x1E는 응답하지 않음)
    uint8_t config_a[] = {0x00, 0x78}; // 75Hz 설정
    auto ret3 = i2c_master_transmit(handle, config_a, 2, pdMS_TO_TICKS(100));
    if(ret3 != ESP_OK) {
        ESP_LOGE("HMC5883L", "HMC5883L 설정 실패");
        return nullptr;
    }

    uint8_t mode_cmd[] = {0x02, 0x00}; // 연속 측정 모드 시작
    auto ret4 = i2c_master_transmit(handle, mode_cmd, 2, pdMS_TO_TICKS(100));
    if(ret4 != ESP_OK) {
        ESP_LOGE("HMC5883L", "HMC5883L 모드 설정 실패");
        return nullptr;
    }

    ESP_LOGI("HMC5883L", "✓ 지자계 센서 설정 완료");
    return handle;
}


/// @brief 
/// @param handle 
/// @return 
inline std::tuple< esp_err_t,std::array<float,3>>read_hmc5883l(i2c_master_dev_handle_t handle){
    
    uint8_t reg = 0x03; // X MSB 레지스터
    uint8_t data[6];    
    esp_err_t ret = i2c_master_transmit_receive(handle, &reg, 1, data, 6, pdMS_TO_TICKS(100));
    if (ret == ESP_OK) {
        // HMC5883L은 Big-Endian: (MSB << 8) | LSB
        // 순서 주의: X(3,4), Z(5,6), Y(7,8)
        int16_t raw_x = (int16_t)((data[0] << 8) | data[1]); 
        int16_t raw_y = (int16_t)((data[2] << 8) | data[3]);    
        int16_t raw_z = (int16_t)((data[4] << 8) | data[5]);    
        return{ESP_OK,{(float)raw_x,(float)raw_y,(float)raw_z}}; 
    }else {
        //ESP_LOGE("HMC5883L", "데이터 읽기 실패!");
        return{ESP_FAIL,{}};
    }     
}

// 지자계 하드 아이언 보정 함수 (사용자에게 8자 운동을 시키면서 최대/최소값을 수집하여 오프셋 계산)
void calibrate_mag_hard_iron(i2c_master_dev_handle_t handle) {
    uint8_t reg = 0x03; // X MSB 레지스터
    uint8_t data[6];    
  
    float mx_max = -99999.0f, mx_min = 99999.0f;
    float my_max = -99999.0f, my_min = 99999.0f;
    float mz_max = -99999.0f, mz_min = 99999.0f;

    ESP_LOGI("HMC5883L", "지자계 보정 시작: 드론을 모든 방향(8자)으로 돌리세요 (약 30초)...");

    // 약 3000번 샘플링 (100Hz 기준 약 30초)
    for (int i = 0; i < 3000; i++) {
        esp_err_t ret = i2c_master_transmit_receive(handle, &reg, 1, data, 6, pdMS_TO_TICKS(100));
        if (ret == ESP_OK) {
            // HMC5883L은 Big-Endian: (MSB << 8) | LSB
            // 순서 주의: X(3,4), Z(5,6), Y(7,8)

            int16_t mx = (int16_t)((data[0] << 8) | data[1]); 
            int16_t my = (int16_t)((data[2] << 8) | data[3]);    
            int16_t mz = (int16_t)((data[4] << 8) | data[5]);    

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
                ESP_LOGI("HMC5883L", "보정 진행 중... (%d%%)", (i * 100) / 3000);
            }
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

    ESP_LOGW("HMC5883L", "보정 완료! 아래 값을 config_ryu.hpp의 MAG_OFFSET에 적용하세요:");
    printf("#define MAG_OFFSET_X %.2f\n", offset_x);
    printf("#define MAG_OFFSET_Y %.2f\n", offset_y);
    printf("#define MAG_OFFSET_Z %.2f\n", offset_z);
    
    // 고급 보정을 위해 스케일 값도 출력
    printf("// Scale Factor (Soft-Iron): X:%.2f, Y:%.2f, Z:%.2f\n", scale_x, scale_y, scale_z);

    while(true){
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}


// // 1. ST1 레지스터(0x10) 주소 준비
// uint8_t st1_addr = 0x09; 
// uint8_t st1_data = 0;

// // 2. 데이터가 준비되었는지 확인 (HMC의 방식과 동일)
// esp_err_t ret_st = i2c_master_transmit_receive(ak_handle, &st1_addr, 1, &st1_data, 1, 10 / portTICK_PERIOD_MS);

// if (ret_st == ESP_OK) {
//     // Bit 0 (DRDY)이 1이면 데이터가 준비된 것입니다.
//     if (st1_data & 0x01) { 
//         // 여기서 실제로 read_ak09916()을 호출하여 데이터를 읽습니다.
//         auto [ret, mag_val] = read_ak09916(ak_handle);
        
//         if (ret == ESP_OK) {
//             // 읽기 성공! mag_val[0], [1], [2] 사용
//         }
//     }
// }
