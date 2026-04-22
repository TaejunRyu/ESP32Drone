#pragma once
#include "ryu_SensorManager.h"


namespace SensorManager {

/**
 * @brief I2C 버스가 락(Hang) 걸렸을 때 수동 클럭을 주어 복구하는 하드웨어 생존 함수
 */
inline void recover_i2c_bus(int scl_io, int sda_io) {
    ESP_LOGW("I2C", "⚠ I2C 버스 락 감지! 하드웨어 강제 복구를 시도합니다.");

    // 1. SCL과 SDA 핀을 일반 GPIO 출력 모드로 변경
    gpio_set_direction((gpio_num_t)scl_io, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)sda_io, GPIO_MODE_INPUT);

    // 2. 센서가 잡고 있는 데이터를 밀어내기 위해 SCL에 9번의 Pulse를 줍니다.
    for (int i = 0; i < 9; i++) {
        gpio_set_level((gpio_num_t)scl_io, 0);
        esp_rom_delay_us(5);
        gpio_set_level((gpio_num_t)scl_io, 1);
        esp_rom_delay_us(5);
    }

    // 3. 복구 신호인 STOP Condition 수동 생성
    gpio_set_direction((gpio_num_t)sda_io, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)sda_io, 0);
    esp_rom_delay_us(5);
    gpio_set_level((gpio_num_t)scl_io, 1);
    esp_rom_delay_us(5);
    gpio_set_level((gpio_num_t)sda_io, 1); // Low -> High (STOP)

    ESP_LOGI("I2C", "✓ I2C 버스 복구 완료. 드라이버를 재초기화 하세요.");
}

/**
 * @brief 두 IMU 데이터를 비교하여 비행에 사용할 최적의 데이터를 선택합니다.
 */
inline std::array<float, 3> get_voted_imu(std::array<float, 3> imu1, std::array<float, 3> imu2) {
    if (imu_status.is_failed) return imu2; // 이미 Primary가 사망했다면 무조건 Backup 반환

    float diff = 0.0f;
    for(int i=0; i<3; i++) diff += fabsf(imu1[i] - imu2[i]);

    if (diff > IMU_DIFF_MAX) {
        imu_status.error_cnt++;
        if (imu_status.error_cnt > ERROR_MAX_LIMIT) {
            ESP_LOGE("SENSOR", "🚨 IMU 0 불량 감지! IMU 1(Backup)로 영구 스위칭합니다.");
            imu_status.active_index = SENSOR_BACKUP;
            imu_status.is_failed = true;
            return imu2;
        }
    } else {
        if (imu_status.error_cnt > 0) imu_status.error_cnt--; // 정상 복귀 시 카운트 차감
    }

    return (imu_status.active_index == SENSOR_PRIMARY) ? imu1 : imu2;
}

/**
 * @brief 지자기 센서 다중화 및 판별
 */
inline std::array<float, 3> get_voted_mag(std::array<float, 3> mag1, std::array<float, 3> mag2) {
    // 지자기는 오차가 크므로 통신 에러 코드로만 스위칭하거나, IIR 필터 후 튐 현상만 체크합니다.
    if (mag_status.is_failed) return mag2;
    return (mag_status.active_index == SENSOR_PRIMARY) ? mag1 : mag2;
}

} // namespace SensorManager


// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "SensorManager.h"
// #include "IST8310.h"
// #include "MahonyFilter.h"

// // [전역 변수] 최종 가공되어 PID 제어기들이 가져다 쓸 공유 데이터들
// struct {
//     std::array<float, 3> acc;
//     std::array<float, 3> gyro;
//     std::array<float, 3> mag;
// } g_voted_sensor;

// void flight_control_task(void *pvParameters) {
//     const float dt = 0.0025f; // 400Hz (2.5ms)
//     uint32_t loop_cnt = 0;

//     // 부팅 초기 1회 캘리브레이션은 편의상 생략 (이미 수립하신 코드 적용)

//     while (1) {
//         // 400Hz 주기를 보장하는 인터럽트 대기 (Task Notify)
//         ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//         loop_cnt++;

//         // =================================================================
//         // [1단계: 독립 데이터 수집] - 센서가 2개씩이므로 각각 다 읽어옵니다.
//         // =================================================================
        
//         // IMU 2개 읽기 (SPI)
//         auto [err_a0, acc0, gyro0] = ICM20948::read_with_offset(imu_handle[0]);
//         auto [err_a1, acc1, gyro1] = ICM20948::read_with_offset(imu_handle[1]);

//         // 지자기 2개 읽기 (I2C) - 20Hz 주기로 읽기 위해 20번(50ms)에 한 번 실행
//         std::array<float, 3> mag0 = {0}, mag1 = {0};
//         if (loop_cnt % 20 == 0) {
//             auto [err_m0, temp_mag0] = IST8310::read_with_offset(mag_handle[0]);
//             auto [err_m1, temp_mag1] = AK09916::read_with_offset(mag_handle[1]);
            
//             // I2C 버스가 락 걸려 둘 다 에러를 뿜는 최악의 상황 감지
//             if (err_m0 != ESP_OK && err_m1 != ESP_OK) {
//                 // I2C 핀 번호를 넣어 복구 함수를 호출합니다.
//                 SensorManager::recover_i2c_bus(22, 21); // 예시 SCL:22, SDA:21
//                 // 복구 후 I2C 드라이버를 다시 initialize 해주는 코드가 여기에 와야 합니다.
//             }
//             mag0 = temp_mag0;
//             mag1 = temp_mag1;
//         }

//         // =================================================================
//         // [2단계: 판별 및 품질 평가] - 센서 매니저를 통해 안전한 데이터만 추출
//         // =================================================================
//         g_voted_sensor.acc  = SensorManager::get_voted_imu(acc0, acc1);
//         g_voted_sensor.gyro = SensorManager::get_voted_imu(gyro0, gyro1);
        
//         if (loop_cnt % 20 == 0) {
//             g_voted_sensor.mag  = SensorManager::get_voted_mag(mag0, mag1);
//         }

//         // =================================================================
//         // [3단계: 제어 루프로 넘기기] - 검증 완료된 1개의 데이터 세트만 전달
//         // =================================================================
        
//         // Mahony 필터 업데이트
//         AHRS::MahonyAHRSupdate(
//             g_voted_sensor.gyro[X], g_voted_sensor.gyro[Y], g_voted_sensor.gyro[Z],
//             g_voted_sensor.acc[X],  g_voted_sensor.acc[Y],  g_voted_sensor.acc[Z],
//             g_voted_sensor.mag[X],  g_voted_sensor.mag[Y],  g_voted_sensor.mag[Z],
//             dt
//         );

//         // -----------------------------------------------------------------
//         // [이후 기존에 올려주신 PID 제어 및 모터 믹싱 로직이 그대로 이어집니다]
//         // -----------------------------------------------------------------
//         if (!g_sys.is_armed) {
//             /* 시동 안 걸렸을 때 로직... */
//         } else {
//             /* 조종기 입력 및 각도 PID(Outer) -> 각속도 PID(Inner) -> 모터 출력... */
//             // 이때 g_imu.gyro[X] 대신 검증된 g_voted_sensor.gyro[X]를 사용하시면 됩니다.
//         }
//     }
// }
