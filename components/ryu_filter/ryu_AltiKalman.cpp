#include "ryu_AltiKalman.h"

namespace Service {

AltiKalman::AltiKalman() {
    reset(0.0f);
}

void AltiKalman::reset(float initial_alt) {
    z = initial_alt;
    v = 0.0f;
    P[0][0] = 1.0f; P[0][1] = 0.0f;
    P[1][0] = 0.0f; P[1][1] = 1.0f;
}

void AltiKalman::predict(float acc_z_earth, float dt) {
    // 1. 상태 예측 (물리 법칙: s = s0 + vt + 0.5at^2)
    z = z + (v * dt) + (0.5f * acc_z_earth * dt * dt);
    v = v + (acc_z_earth * dt);

    // 2. 공분산 예측 (P = FPF' + Q)
    P[0][0] += dt * (P[1][0] + P[0][1] + dt * P[1][1]) + Q_accel;
    P[0][1] += dt * P[1][1];
    P[1][0] += dt * P[1][1];
    P[1][1] += Q_accel;
}

void AltiKalman::update(float baro_alt) {
    // 3. 칼만 이득 계산
    float S = P[0][0] + R_baro;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // 4. 상태 보정 (측정 오차 반영)
    float y = baro_alt - z;
    z += K[0] * y;
    v += K[1] * y;

    // 5. 공분산 보정
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;
}

} // namespace Service
