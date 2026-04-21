#pragma once

#include <driver/i2c_master.h>
#include <esp_log.h>
#include <esp_timer.h>
#include "ryu_config.h"
namespace ICM20948
{
//ADO핀을 바꾸면 BMP388의 주소도 같이 다른번호로 바뀌는것 주의 
inline constexpr uint8_t  ADDR_GND =  0x68; // AD0핀이 GND일 때 0x68, VCC일 때 0x69
inline constexpr uint8_t  ADDR_VCC =  0x69; // AD0핀이 GND일 때 0x68, VCC일 때 0x69

/**
 * @brief 
 *      icm20948초기화 함수
 * @param bus_handle    i2c bus handle
 * @param dev_address   icm20948의 주소
 * @return i2c_master_dev_handle_t 
 */
extern i2c_master_dev_handle_t initialize(i2c_master_bus_handle_t bus_handle, uint16_t dev_address);

/**
 * @brief
 *      bypass 활성화
 * @param handle 이 핸들에 관련되어진 bypass 설정
 * @return esp_err_t
 */
extern esp_err_t enable_mag_bypass(i2c_master_dev_handle_t handle);


/**
 * @brief icm20948의 raw값만을 처리한다
 *
 * @param handle
 * @return std::tuple<esp_err_t, std::array<float, 3>, std::array<float, 3>>
 */
extern "C" std::tuple<esp_err_t, std::array<float, 3>, std::array<float, 3>> read_raw_data(i2c_master_dev_handle_t handle);


/**
 * @brief icm20948의 캘리브레이션을 실행한다.
 *        이센서의 offset을 구하기 위하여 시스템 시작시 한번은 실행하여 offset을 구해야 한다.
 * @param handle 
 * @return std::tuple<std::array<float, 3>, std::array<float, 3>> 
 */
extern std::tuple<std::array<float, 3>, std::array<float, 3>> calibrate(i2c_master_dev_handle_t handle);

/**
 * @brief  원시 데이터의 모든 설정값을 조건에 맞게 설정하여 반환한다. 
 *      
 * @param handle       
 * @param offset_acc    시스템 시작시 캘리브레이션으로 얻은 acc  offset이다.
 * @param offset_gyro   시스템 시작시 캘리브레이션으로 얻은 gyro offset이다.
 * @return std::tuple < esp_err_t , std::array<float,3>, std::array<float,3>> 
 */
inline std::tuple < esp_err_t , std::array<float,3>, std::array<float,3>> 
        read_with_offset(i2c_master_dev_handle_t handle,std::array<float,3> offset_acc,std::array<float,3> offset_gyro){
    auto [ret,acc,gyro] = read_raw_data(handle);
    acc[X]  = acc[X]  - offset_acc[X];
    acc[Y]  = acc[Y]  - offset_acc[Y];
    acc[Z]  = acc[Z]  - offset_acc[Z];
    gyro[X] = gyro[X] - offset_gyro[X];
    gyro[Y] = gyro[Y] - offset_gyro[Y];
    gyro[Z] = gyro[Z] - offset_gyro[Z];

    // ICM20948에서 부호를 반대로 설정해놨어요. 논리 대로 처리 간다~~~~  
    // qgc의 roll , pitch에 (-) 부호처리 일단함.
    // 오른손법칙에 어긋나는 부분 교정하여 Mahony에 입력한다 
    acc[Y]  *=  -1.0f;
    gyro[X] *=  -1.0f;

    // 오른쪽으로 회전시 (-)부호로 값은 커진다. (즉 값이 작아진다는 것이다)
    gyro[Z] *=  -1.0f;
    return {ret,acc,gyro};
}
}