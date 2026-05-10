#pragma once

#include <cmath>

namespace Filter {

class AltiKalman {
private:
    AltiKalman();
    ~AltiKalman() = default;

public:
    static AltiKalman& get_instance() {
        static AltiKalman instance;
        return instance;
    }

    // 상태 벡터: x = [고도(m), 수직속도(m/s)]
    float z = 0.0f;
    float v = 0.0f;

    // 공분산 행렬 (2x2)
    float P[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};

    // 프로세스 노이즈 (가속도계의 불확실성)
    const float Q_accel = 0.01f;
    // 측정 노이즈 (기압계의 노이즈 - 높을수록 기압계를 덜 믿음)
    const float R_baro = 0.5f; 

    /**
     * @param acc_z_earth 중력 가속도가 제거된 순수 수직 가속도 (m/s^2)
     * @param dt 루프 주기 (sec)
     */
    void predict(float acc_z_earth, float dt);

    /**
     * @param baro_alt 기압계로부터 계산된 현재 고도 (m)
     */
    void update(float baro_alt);

    void reset(float initial_alt);
};

} // namespace Service



// // AHRS 칼만 필터에서 얻은 쿼터니언(q)을 활용
// float q0 = kalman.x[0], q1 = kalman.x[1], q2 = kalman.x[2], q3 = kalman.x[3];

// // 1. 기체 좌표계의 가속도를 지구 좌표계로 변환 (수직 성분만 추출)
// float acc_z_earth = 2.0f*(q1*q3 - q0*q2)*ax + 2.0f*(q0*q1 + q2*q3)*ay + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*az;

// // 2. 중력 가속도(1.0G) 제거 (단위가 m/s^2라면 9.81을 뺌)
// // 센서값이 G 단위라면 1.0f를 뺌
// acc_z_earth -= 1.0f; 

// // 3. 고도 칼만 업데이트
// auto& alti = Service::AltiKalman::get_instance();
// alti.predict(acc_z_earth * 9.81f, dt); // m/s^2 단위로 변환해서 입력
// if (baro_updated) {
//     alti.update(current_baro_alt);
// }
