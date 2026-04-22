#include <array>
#include <cmath>
#include <esp_log.h>
#include <driver/gpio.h>
#include <esp_rom_sys.h> // 딜레이용

namespace SensorManager {

enum SensorSource { SENSOR_PRIMARY = 0, SENSOR_BACKUP = 1 };

struct SensorHealth {
    SensorSource active_index = SENSOR_PRIMARY;
    int error_cnt = 0;
    bool is_failed = false;
};

// 각 센서군의 상태 전역 관리
inline SensorHealth imu_status;
inline SensorHealth mag_status;

// 불일치(Inconsistency) 판별을 위한 임계값
inline constexpr float IMU_DIFF_MAX = 5.0f; // 가속도/자이로 차이 임계값
inline constexpr int ERROR_MAX_LIMIT = 10;  // 10번 연속 이상 시 스위칭
}
