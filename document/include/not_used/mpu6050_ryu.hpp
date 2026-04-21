#pragma once

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "config.hpp"


static constexpr uint8_t MPU6050_DEVICE_ADDRESS = 0x68;         // ADO PIN: GND
static constexpr uint8_t MPU6050_SECOND_DEVICE_ADDRESS = 0x69;  // ADO PIN : 3.3V

static constexpr uint8_t MPU6050_DATA_ADDRESS   = 0x3B;

/**
 * @brief 
 * mpu6050의 초기화 
 * @param bus_handle    초기화 완료되어진 버스 핸들 
 * @param dev_address   가속도/자이로 센서의 주소
 * @return i2c_master_dev_handle_t (센서 디바이스 핸들) 
 */
i2c_master_dev_handle_t init_mpu6050(i2c_master_bus_handle_t bus_handle,  uint16_t dev_address){
    
    i2c_master_dev_handle_t handle = nullptr;

    i2c_device_config_t device_cfg = {};
    device_cfg.dev_addr_length=I2C_ADDR_BIT_LEN_7; 
    device_cfg.device_address= dev_address;     // AD0 핀이 GND면 0x68, VCC면 0x69
    device_cfg.scl_speed_hz=I2C_SPEED;                     // MPU6050은 Fast Mode(400kHz) 지원
    
    // ESP-IDF v5.x의 I2C Master Driver를 사용하여 센서를 버스에 등록
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &device_cfg, &handle)); 
    
    // MPU6050을 깨우고(Sleep 해제) 클럭 소스를 설정
    ESP_ERROR_CHECK(i2c_master_transmit(handle, (uint8_t[]){0x6B, 0x01}, 2, pdMS_TO_TICKS(100)));
    vTaskDelay(pdMS_TO_TICKS(50)); // 안정화를 위한 짧은 대기
/**
 * DLPF 설정 값 (Register 0x1A)
 * 0: 260Hz (지연 최소, 노이즈 최대)
 * 1: 184Hz
 * 2: 94Hz
 * 3: 44Hz  
 * 4: 21Hz  <-- 400Hz 루프에서 가장 추천 (진동이 많을 때)
 * 5: 10Hz
 * 6: 5Hz   <-- 지연 시간 매우 큼 (드론 추락 위험)
 */
    // MPU6050의 DLPF를 설정하는 코드입니다.
    // MPU6050의 Register 0x1A (CONFIG) 값을 변경하여 설정하며, 400Hz 루프에 가장 적합한 21Hz 대역폭 설정
    ESP_ERROR_CHECK(i2c_master_transmit(handle, (uint8_t[]){0x1A, 0x04}, 2, pdMS_TO_TICKS(100)));
    
    // 자이로스코프 설정 +-1000dps
    ESP_ERROR_CHECK(i2c_master_transmit(handle, (uint8_t[]){0x1B, 0x10}, 2, pdMS_TO_TICKS(100)));
    
    // 가속도계 설정  +-8g 
    ESP_ERROR_CHECK(i2c_master_transmit(handle, (uint8_t[]){0x1C, 0x10}, 2, pdMS_TO_TICKS(100)));
    
    return handle;
}

/**
 * @brief Set the bypass mode object
 * 
 * @param handle 
 * @return esp_err_t 
 */
esp_err_t mpu6050_enable_mag_bypass(i2c_master_dev_handle_t handle){
    // Bypass Mode 활성화 (핵심!)
    esp_err_t ret = i2c_master_transmit(handle, (uint8_t[]){0x37, 0x02}, 2, pdMS_TO_TICKS(100));
    return ret;
}


/**
 * @brief 
 * 
 * @param handle 
 * @return std::tuple<std::array<float,3>,std::array<float,3>> 
 */
std::tuple<std::array<float,3>,std::array<float,3>> calibrate_mpu6050( i2c_master_dev_handle_t handle) {

    std::array<float,3> sum_acc={};
    std::array<float,3> sum_gyro={};
     int valid_samples = 0; // 실제로 성공한 샘플 수만 카운트
    const int samples = 600; // 500번 샘플링 (약 1~2초 소요)
    uint8_t buf[14];
    
    ESP_LOGI("MPU6050", "센서 영점 조절 시작... 기체를 수평으로 유지하세요.");
    for (int i = 0; i < samples; i++) {
        if (i2c_master_transmit_receive(handle, &MPU6050_DATA_ADDRESS, 1, buf, 14, pdMS_TO_TICKS(5)) == ESP_OK) {    
             if (buf[8] == 0xFF && buf[9] == 0xFF) continue;         
            sum_acc[0] += (int16_t)((buf[0] << 8) | buf[1])   / 4096.0f;
            sum_acc[1] += (int16_t)((buf[2] << 8) | buf[3])   / 4096.0f;
            sum_acc[2] += (int16_t)((buf[4] << 8) | buf[5])   / 4096.0f;
            // 온도 (6~7) - 건너뜀
            sum_gyro[0] += (int16_t)((buf[8] << 8) | buf[9])   / 32.8f;  // x축과 y축이 바뀜.
            sum_gyro[1] += (int16_t)((buf[10] << 8) | buf[11]) / 32.8f;
            sum_gyro[2] += (int16_t)((buf[12] << 8) | buf[13]) / 32.8f;
            valid_samples++;
        }       
        //ESP_LOGI("MPU6050", "센서 영점 조절 작업중.%d/%d",i,samples);
        vTaskDelay(pdMS_TO_TICKS(10)); // 250Hz 주기로 샘플링
    }
    //printf("\n");
    if(valid_samples > 0 ){
        // 평균값 계산 (Offset 저장)
        sum_acc[0]  = sum_acc[0] / valid_samples;
        sum_acc[1]  = sum_acc[1] / valid_samples;
        sum_acc[2]  = (sum_acc[2] / valid_samples) -1.0f;  // 가속도는 중력 가속도(1g)를 제외하고 0이 되어야 함m

        sum_gyro[0] = sum_gyro[0] / valid_samples;
        sum_gyro[1] = sum_gyro[1] / valid_samples;
        sum_gyro[2] = sum_gyro[2] / valid_samples;
    }else{
        sum_acc ={}; 
        sum_gyro={};
    }
    return {sum_acc,sum_gyro};
}



/**
 * @brief 
 * 
 * @param handle 
 * @param offset_acc 
 * @param offset_gyro 
 * @return std::tuple < esp_err_t , std::array<float,3>, std::array<float,3>> 
 */
inline std::tuple < esp_err_t , std::array<float,3>, std::array<float,3>> 
        read_mpu6050(i2c_master_dev_handle_t handle ,
                        const std::array<float,3> offset_acc,
                        const std::array<float,3> offset_gyro){
    
    uint8_t buf[14];

    esp_err_t ret = i2c_master_transmit_receive(handle, &MPU6050_DATA_ADDRESS, 1, buf, 14, pdMS_TO_TICKS(5));
    if(ret != ESP_OK){
        return{ret,{},{}};
    } 
    float macc[3] ={},mgyro[3] ={},raw_acc[3] ={},raw_gyro[3]={};
    if(handle == imu_handle || handle == imu_sub_handle){
        // 센서 데이터 변환 (가속도, 자이로)
        // 1. 가속도 전체 반전 (들어올린 쪽이 +가 되도록)
        // 단위 변환 (기본값: Accel ±8g, Gyro ±1000dps 기준)
        raw_acc[0] = (static_cast<int16_t>((buf[0] << 8) | buf[1]) / 4096.0f);
        raw_acc[1] = (static_cast<int16_t>((buf[2] << 8) | buf[3]) / 4096.0f);
        raw_acc[2] = (static_cast<int16_t>((buf[4] << 8) | buf[5]) / 4096.0f);
        // 2. 자이로도 가속도와 세트로 반전 (회전 방향 일치)
        raw_gyro[0] = (static_cast<int16_t>((buf[8]  << 8) | buf[9])  / 32.8f); 
        raw_gyro[1] = (static_cast<int16_t>((buf[10] << 8) | buf[11]) / 32.8f);
        raw_gyro[2] = (static_cast<int16_t>((buf[12] << 8) | buf[13]) / 32.8f); // z축이 반대가,,,,
    }

    // 센서 데이터 변환 (가속도, 자이로)
    macc[0]  =(raw_acc[0]   - offset_acc[0] ); 
    macc[1]  =(raw_acc[1]   - offset_acc[1] );
    macc[2]  =(raw_acc[2]   - offset_acc[2] );
    mgyro[0] =(raw_gyro[0]  - offset_gyro[0]);
    mgyro[1] =(raw_gyro[1]  - offset_gyro[1]);
    mgyro[2] =(raw_gyro[2]  - offset_gyro[2]);

    return {ret,{macc[0],macc[1],macc[2]},{mgyro[0],mgyro[1],mgyro[2]}};
}  