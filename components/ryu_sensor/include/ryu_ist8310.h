#pragma once
 
#include <tuple>
#include <driver/i2c_master.h>
#include <esp_log.h>
#include "ryu_config.h"
namespace IST8310
{
// IST8310 레지스터 정의
inline constexpr uint8_t ADDR       =    0x0E; // 기본 주소 (ADR핀 상태에 따라 다를 수 있음)
inline constexpr uint8_t WHO_AM_I   =    0x00; // ID 확인용 (값: 0x10)
inline constexpr uint8_t STAT1      =    0x02; // 데이터 준비 상태 (Bit 0: DRDY)
inline constexpr uint8_t DATA_X_L   =    0x03; // 데이터 시작 (X-axis Low)
inline constexpr uint8_t CONTROL1   =    0x0A; // 모드 설정 (Single/Continuous)
inline constexpr uint8_t CONTROL2   =    0x0B; // 소프트 리셋 및 옵션
inline constexpr uint8_t AVGCNTL    =    0x41; // 평균 필터 설정 (노이즈 감소)
inline constexpr uint8_t PDCNTL     =    0x42; // Pulse Duration 제어
inline constexpr uint8_t CROSSAXIS1 =    0x48; // 
inline constexpr uint8_t CROSSAXIS2 =    0x49; // 
// 센서 감도: 1320 LSB/Gauss (0.3 µT/LSB)
inline constexpr float SENSITIVITY   =  0.3f;   
inline constexpr float MAG_MAX_X     =  58.20f;
inline constexpr float MAG_MAX_Y     =  50.40f;
inline constexpr float MAG_MAX_Z     =  29.10f;

inline constexpr float MAG_MIN_X     =  -34.50f;
inline constexpr float MAG_MIN_Y     =  -41.70f;
inline constexpr float MAG_MIN_Z     =  -59.70f;

inline constexpr float MAG_OFFSET_X  =  11.85f;
inline constexpr float MAG_OFFSET_Y  =  4.35f;
inline constexpr float MAG_OFFSET_Z  =  -15.30f;

inline constexpr float SCALE_X       =  0.98f;
inline constexpr float SCALE_Y       =  0.99f;
inline constexpr float SCALE_Z       =  1.03f;

extern i2c_master_dev_handle_t initialize(i2c_master_bus_handle_t bus_handle);
std::tuple<esp_err_t, std::array<float, 3>> read_raw_data(i2c_master_dev_handle_t handle);
extern void calibrate_hard_iron(i2c_master_dev_handle_t handle);

/**
 * @brief 저장되어진 offset를 적용함.
 * 
 * @param handle 
 * @return std::tuple<esp_err_t, std::array<float, 3>> 
 */
inline std::tuple<esp_err_t, std::array<float, 3>> read_with_offset(i2c_master_dev_handle_t handle) {
    auto [ret,mag_data] = read_raw_data(handle);
    mag_data[X] = (mag_data[X]- MAG_OFFSET_X) * SCALE_X;
    mag_data[Y] = (mag_data[Y]- MAG_OFFSET_Y) * SCALE_Y;
    mag_data[Z] = (mag_data[Z]- MAG_OFFSET_Z) * SCALE_Z;

    float norm = sqrtf(mag_data[X] * mag_data[X] + mag_data[Y] * mag_data[Y] + mag_data[Z] * mag_data[Z]);
    if (norm > 0.0f) {
        mag_data[X] =mag_data[X]/norm;
        mag_data[Y] =mag_data[Y]/norm;
        mag_data[Z] =mag_data[Z]/norm;
    }
    
    // X를 (-)부호를 해야지 Mahony를 통과
    mag_data[X] *=  -1.0f;
    return {ret,mag_data};
}

/**
 * @brief 데이터가 준비 되었있는가?
 * 
 * @param handle 
 * @return std::tuple<esp_err_t,uint8_t> 
 */
inline std::tuple<esp_err_t,uint8_t> ready_data(i2c_master_dev_handle_t handle){
    uint8_t reg_addr = STAT1;
    uint8_t data = 0x00;
    esp_err_t ret_st  = i2c_master_transmit_receive(handle, &reg_addr, 1, &data, 1,  pdMS_TO_TICKS(2));
    return {ret_st,data};
}

}