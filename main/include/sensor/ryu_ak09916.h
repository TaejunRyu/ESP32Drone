#pragma once

#include <driver/i2c_master.h>
#include <esp_log.h>
#include "ryu_config.h"
namespace AK09916
{
// AK09916의 주소는 영구적으로 고정되어 있음.    
// AK09916 레지스터 정의
inline constexpr uint8_t ADDR       =    0x0C;
inline constexpr uint8_t STAT1      =    0x10;    // 데이터의 준비 체크

inline constexpr float SCALE_X      =   0.91f;
inline constexpr float SCALE_Y      =   1.07f;
inline constexpr float SCALE_Z      =   1.04f;
inline constexpr float MAG_OFFSET_X =   91.50f;
inline constexpr float MAG_OFFSET_Y =   176.00f;
inline constexpr float MAG_OFFSET_Z =   -170.50f;

extern i2c_master_dev_handle_t initialize(i2c_master_bus_handle_t bus_handle);

extern std::tuple<esp_err_t, std::array<float, 3>> read_data(i2c_master_dev_handle_t handle);

extern void calibrate_hard_iron(i2c_master_dev_handle_t handle);

/**
 * @brief   offset이 적용되어진 값을 읽어온다 
 * 
 * @param handle 
 * @return std::tuple<esp_err_t, std::array<float, 3>> 
 */
inline std::tuple<esp_err_t, std::array<float, 3>> read_with_offset(i2c_master_dev_handle_t handle) {
    auto [ret,mag_data] = read_data(handle);
    mag_data[0] = (mag_data[X]- MAG_OFFSET_X) * SCALE_X;
    mag_data[1] = (mag_data[Y]- MAG_OFFSET_Y) * SCALE_Y;
    mag_data[2] = (mag_data[Z]- MAG_OFFSET_Z) * SCALE_Z;

    float norm = sqrtf(mag_data[X] * mag_data[X] + mag_data[Y] * mag_data[Y] + mag_data[Z] * mag_data[Z]);
    if (norm > 0.0f) {
        mag_data[X] =mag_data[X]/norm;
        mag_data[Y] =mag_data[Y]/norm;
        mag_data[Z] =mag_data[Z]/norm;
    }

    //
    float cal_x = mag_data[X] * -1.0f;
    float cal_y = mag_data[Y] * -1.0f;
    float cal_z = mag_data[Z];// * -1.0f;

    return {ret,{cal_x,cal_y,cal_z}};
}

/**
 * @brief 읽어올 데이터가 준비 되었는가?
 * 
 * @param handle 
 * @return std::tuple<esp_err_t,uint8_t> 
 */
inline std::tuple<esp_err_t,uint8_t> ready_data(i2c_master_dev_handle_t handle){
    uint8_t data = 0x00;
    esp_err_t ret_st  = i2c_master_transmit_receive(handle, &STAT1, 1, &data, 1,  pdMS_TO_TICKS(1));
    return {ret_st,data};
}

// 루프 안에서 실시간으로 범위를 찾는 방식
// void autoCalibrate(float rawX, float rawY, float rawZ) {
//   // 움직임 감지 (가속도 센서 활용 권장)
//   if (isMoving()) { 
//     if (rawX > magMaxX) magMaxX = rawX;
//     if (rawX < magMinX) magMinX = rawX;
//     // Y, Z 축도 동일하게 처리
    
//     // 오프셋 계산
//     offsetX = (magMaxX + magMinX) / 2.0;
//     scaleX = 1000.0 / (magMaxX - magMinX); // 정규화 스케일
//   }
// }



}