#include "ryu_SensorTask.h"

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h> // 시간 측정용 (선택사항)

//#include "sensor/ryu_i2c.h"
// 첫번째 센서로 
// icm20948, ak09916, bmp388 위의 센서 교체.
#include "ryu_icm20948.h"
#include "ryu_ak09916.h"
#include "ryu_bmp388.h"   

// gps에 있는 지자계센서.
#include "ryu_ist8310.h"

namespace SENSOR{

    // 전역 변수로 선언하거나 태스크에 포인터로 전달
    SensorSharedData shared_data;
    SemaphoreHandle_t sensor_imu_mutex;  // 뮤텍스 핸들
    SemaphoreHandle_t sensor_mag_mutex;  // 뮤텍스 핸들
    SemaphoreHandle_t sensor_baro_mutex; // 뮤텍스 핸들

// 1ms 주기를 틱(Tick) 단위로 변환
#define SENSOR_PERIOD_MS 1

const TickType_t xFrequency = pdMS_TO_TICKS(SENSOR_PERIOD_MS);
static int loop_count = 0;

void SensorRead_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while(true) {
        // [1] 센서 데이터 읽기 (로컬 변수에 먼저 저장)
        // IMU 1, 2 읽기
        auto [ret1, macc1, mgyro1] = Sensor::ICM20948::Main().read_with_offset();
        auto [ret2, macc2, mgyro2] = Sensor::ICM20948::Sub().read_with_offset();

        // [2] 센서 조합 (단순 평균)
        std::array<float, 3> avg_acc ={}, avg_gyro={};
        for(int i=0; i<3; i++) {
            avg_acc[i] = (macc1[i] + macc2[i]) * 0.5f;
            avg_gyro[i] = (mgyro1[i] + mgyro2[i]) * 0.5f;
        }
        // [3] 필터 적용 (esp-dsp 호출)
        // ※ 실제 구현시에는 미리 생성된 coeffs를 사용하세요.
        std::array<float, 3> f_acc = {}, f_gyro ={};
        for(int i=0; i<3; i++) {
            // 한 샘플씩 처리 (len=1)
            // dsps_biquad_f32(&avg_acc[i], &f_acc[i], 1, acc_coeffs, acc_w[i]);
            // dsps_biquad_f32(&avg_gyro[i], &f_gyro[i], 1, gyro_coeffs, gyro_w[i]);
        }

        // [2] 뮤텍스 락 - 데이터를 공유 변수에 옮기기 직전에 잠금
        if (xSemaphoreTake(sensor_imu_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {   
            // IMU 데이터 업데이트 (오타 수정됨)
            shared_data.imu.acc1 = macc1;
            shared_data.imu.gyro1 = mgyro1;
            shared_data.imu.acc2 = macc2;
            shared_data.imu.gyro2 = mgyro2;
            shared_data.imu.filtered_acc = f_acc;   // 필터링된 결과물
            shared_data.imu.filtered_gyro = f_gyro; // 필터링된 결과물
            shared_data.imu.timestamp = esp_timer_get_time();
            xSemaphoreGive(sensor_imu_mutex); // 언락
        }

        // 기압계/지자기 조건부 업데이트
        if (loop_count % 20 == 0) {
            auto [retM1, mag1] = Sensor::IST8310::get_instance().read_with_offset();
            auto [retM2, mag2] = AK09916::read_with_offset(mag_handle[1]);
            if (xSemaphoreTake(sensor_mag_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {   
                shared_data.mag.mag1 = mag1;
                shared_data.mag.mag2 = mag2;
                xSemaphoreGive(sensor_mag_mutex); // 언락
            }
        }

        if (loop_count % 100 == 0) {
            auto [retP1, p1] = Sensor::BMP388::Main().get_relative_altitude();
            auto [retP2, p2] = Sensor::BMP388::Sub().get_relative_altitude();
            if (xSemaphoreTake(sensor_baro_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {   
                shared_data.baro.pressure1 = p1;
                shared_data.baro.pressure2 = p2;
                xSemaphoreGive(sensor_baro_mutex); // 언락
            }
        }

        loop_count = (loop_count + 1) % 1000;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


float acc_coeffs[5];   // 가속도용 계수 저장 (b0, b1, b2, a1, a2)
float gyro_coeffs[5];  // 자이로용 계수 저장

void setup_filters() {
    float sampling_freq = 1000.0f; // 1ms 주기로 읽으므로 1000Hz
    float q_factor = 0.707f;       // 필터의 날카로운 정도 (0.707이 표준)

    // 1. 가속도 필터:  진동을 꽉 잡기 위해 낮게 설정 (예: 20Hz) 
    // 1.1 진동이 심한 드론이라면 acc_cutoff를 낮추세요 (15Hz ~ 20Hz).
    // 1.2 조종이 둔하다면 gyro_cutoff를 높이세요 (100Hz 이상).
    float acc_cutoff = 20.0f;
    //dsps_biquad_gen_lpf_f32(acc_coeffs, acc_cutoff / sampling_freq, q_factor);

    // 2. 자이로 필터: 반응 속도를 위해 조금 높게 설정 (예: 80Hz)
    float gyro_cutoff = 80.0f;
    //dsps_biquad_gen_lpf_f32(gyro_coeffs, gyro_cutoff / sampling_freq, q_factor);
}

}//namespace SENSOR

