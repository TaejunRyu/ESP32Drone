#pragma once

#include <cmath>
#include <algorithm>
#include <esp_log.h>
#include <esp_timer.h>

namespace Filter {

class KalmanFilter {
private:
    // 싱글톤 설계를 위해 생성자 제한
    KalmanFilter();
    ~KalmanFilter() = default;
    
    static constexpr const char* TAG = "KalmanFilter";

public:
    // 복사 및 이동 금지
    KalmanFilter(const KalmanFilter&) = delete;
    KalmanFilter& operator=(const KalmanFilter&) = delete;

    static KalmanFilter& get_instance() {
        static KalmanFilter instance;
        return instance;
    }

    // 상태 벡터 x = [q0, q1, q2, q3, bx, by, bz]
    float x[7] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    float& gyro_x_err = x[4]; 
    float& gyro_y_err = x[5]; 
    float& gyro_z_err = x[6]; 

    // 오차 공분산 행렬 P (7x7)
    float P[7][7] = {0.0f,};

    // 필터 게인 파라미터 (S3 환경에 맞춘 튜닝값)
    const float Q_quat  = 0.001f;   // 쿼터니언 변화 노이즈
    const float Q_bias  = 0.000001f; // 자이로 바이어스 변화 노이즈
    const float R_accel = 0.05f;    // 가속도계 신뢰도 (낮을수록 가속도계를 더 믿음)

    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);
    void get_euler(float* roll, float* pitch, float* yaw);
    void get_speed_err(float* rollspeed, float* pitchspeed, float* yawspeed);
    void reset();

private:
    const float R_mag = 0.2f; // 지자계 신뢰도 (주변 금속 물질이 많으면 값을 높이세요)
    void update_mag(float mx, float my, float mz);
    void predict(float gx, float gy, float gz, float dt);
    void update_accel(float ax, float ay, float az);
    void normalize_quat();
};

} // namespace Service




// void Controller::Flight::flight_task(void* pvParameters) {
//     auto& kalman = Service::KalmanFilter::get_instance();
//     // ... 센서 데이터 읽기 (gx, gy, gz, ax, ay, az) ...

//     // 칼만 필터 업데이트
//     kalman.update(gx, gy, gz, ax, ay, az, dt);

//     // 오일러 각 가져오기
//     float r, p, y;
//     kalman.get_euler(&r, &p, &y);
// }




// [수정 전] mahony.calibrate_mahony_initial_attitude(acc[0], acc[1], acc[2], magx, magy, magz);

// [수정 후] 
// auto& kalman = Service::KalmanFilter::get_instance();
// kalman.reset(); // 공분산 및 상태 초기화

// 초기 자세 설정 (기존 Mahony에서 쓰던 로직과 동일하게 쿼터니언 초기값을 잡아주는 것이 좋습니다)
// 만약 KalmanFilter 클래스에 별도의 초기 자세 설정 함수를 만들지 않았다면 
// 아래와 같이 직접 쿼터니언 초기값을 대입하거나 초기 2~3초간 가만히 두어 수렴시킵니다.



// void Controller::Flight::flight_task(void* pvParameters) {
//     auto& kalman = Service::KalmanFilter::get_instance();
//     // ... 센서 데이터 읽기 코드 (acc, gyro, mag) ...

//     // [수정 전] mahony.MahonyAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
    
//     // [수정 후] 칼만 필터 업데이트
//     kalman.update(gx, gy, gz, ax, ay, az, mx, my, mz, dt);

//     // 자세 데이터(Euler Angle) 가져오기
//     float roll, pitch, yaw;
//     kalman.get_euler(&roll, &pitch, &yaw);

//     // 이제 이 roll, pitch, yaw를 PID 제어기의 입력으로 사용합니다.
// }




//esp32s3로 교체 되었을경우 사용
// #pragma once

// #include <cmath>
// #include <algorithm>
// #include "esp_dsp.h"

// namespace Service {

// class KalmanFilter {
// private:
//     KalmanFilter();
//     ~KalmanFilter() = default;

//     static constexpr const char* TAG = "EKF_S3";

//     // 7-상태 벡터: [q0, q1, q2, q3, bx, by, bz]
//     float x[7] __attribute__((aligned(16)));
//     // 7x7 오차 공분산 행렬
//     float P[49] __attribute__((aligned(16)));
//     // 7x7 시스템 자코비안 행렬
//     float F[49] __attribute__((aligned(16)));
//     // 7x7 프로세스 노이즈 행렬
//     float Q[49] __attribute__((aligned(16)));

// public:
//     static KalmanFilter& get_instance() {
//         static KalmanFilter instance;
//         return instance;
//     }

//     void reset();
//     void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
//     void get_euler(float* roll, float* pitch, float* yaw);

// private:
//     void predict(float gx, float gy, float gz, float dt);
//     void update_accel(float ax, float ay, float az);
//     void update_mag(float mx, float my, float mz);
//     void normalize_quat();
// };

// } // namespace Service
