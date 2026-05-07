#pragma once

#include <nvs_flash.h>
#include <nvs.h>


/**
 * @brief NVS에 저장할 항목정리
 *  1. IMU의 Calibration 정보
 *      1.1 acc  offset
 *      1.2 gyro offset
 *      1.3 mag  offset
 *  2. pid 정보
 * 
 *  3. qgc와 연결되어진 변수 
 * 
 *  4.
 * 
 */
namespace NVS
{
inline constexpr char NVS_NAMESPACE[] = "QGC_PARAMS";

struct SensorCalibration {
    float mag_main_offset[3];   // IST8310 Hard-iron 보정값
    float mag_sub_offset[3];    // AK09916 Hard-iron 보정값
    bool is_calibrated;         // 캘리브레이션 완료 여부 플래그
};

}//namespace NVS

