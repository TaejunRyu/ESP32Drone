#pragma once

#include <esp_pm.h>
#include <driver/i2c_master.h>
#include <driver/mcpwm_prelude.h> // 신형 MCPWM
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ryu_config.h"
#include "ryu_wifi.h"
#include "ryu_gps.h"
#include "ryu_flysky.h"
#include "ryu_telemetry.h"
#include "ryu_MahonyFilter.h"
#include "ryu_pid.h"
#include "ryu_battery.h"
#include "ryu_buzzer.h"
#include "ryu_common_std.h"
#include "ryu_error_proc.h"

#include "sensor/ryu_i2c.h"
// 첫번째 센서로 
// icm20948, ak09916, bmp388 위의 센서 교체.
#include "sensor/ryu_icm20948.h"
#include "sensor/ryu_ak09916.h"
#include "sensor/ryu_bmp388.h"   

// gps에 있는 지자계센서.
#include "sensor/ryu_ist8310.h"
#include "ryu_servo.h"



/* 
 * [ICM-20948 AHRS Configuration Log]
 * 1. Coord: X-Forward, Y-Right, Z-Up
 * 2. Mag Mapping: mx = raw_mx, my = raw_my, mz = raw_mz 
 * 3. Calibration: Applied Hard-Iron Offsets (0-centered)
 * 4. Filter: Mahony AHRS (Kp=2.0, Ki=0.005)
 * 5. Output: Yaw is True North (Declination +7.7deg applied)
 * 
    <<<방위별 표준 데이터 변화 (X축 전방 기준)>>>
    방위 (Heading)	기기 정면 방향	    (mx)	        (my)	        비고
    북 (North, 0°)	    북쪽	        최대값 (+)	    0 근처	        mx가 가장 큼
    동 (East, 90°)	    동쪽	        0 근처	        최대값 (+)	    my가 가장 큼
    남 (South, 180°)    남쪽	        최소값 (-)	    0 근처	        mx가 가장 작음
    서 (West, 270°)	    서쪽	        0 근처	        최소값 (-)	    my가 가장 작음
 */

/**
 * @brief 
 * 어떤 단위의 작업이 얼마나 걸리는지 체크하는 class 실제 모듈에서는 빠져도 됨
 * 
 */
namespace FLIGHT{

inline constexpr uint8_t  ERROR_MAX_NUM  = 10; //최대 에러 발생 한계값
inline constexpr uint8_t  ERROR_CNT_NUM  =  3; //연속적인 에러 발생 수. (발생수가 넘으면 counting 한다) 
inline constexpr  float    dt          = 0.0025f;  // 400hz
inline constexpr  uint64_t INTERVAL_US = 2500ULL;

extern uint64_t  total_us;
extern float calculated_dt;

extern uint8_t imu_error_cnt;
extern uint8_t mag_error_cnt;
extern uint8_t baro_error_cnt;

extern uint8_t imu_active_index;
extern uint8_t mag_active_index;
extern uint8_t baro_active_index;

class PerfMonitor {
    using clock = std::chrono::high_resolution_clock;
    clock::time_point start_time;
public:
    // 측정 시작
    void start() noexcept {
        start_time = clock::now();
    }
    [[nodiscard]] int64_t stop() const noexcept {
        const auto end_time = clock::now(); // const로 명시
        const auto diff = end_time - start_time; // 연산을 분리하여 IntelliSense 추론 도움
        return std::chrono::duration_cast<std::chrono::microseconds>(diff).count();
    }
};

extern class PerfMonitor part_timer;
extern void flight_task(void *pv);
}