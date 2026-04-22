#pragma once

namespace KALMAN{

struct Kalman_t{
    float Q_angle;   // 가속도계 프로세스 노이즈 (기본값: 0.001f)
    float Q_bias;    // 자이로 바이어스 프로세스 노이즈 (기본값: 0.003f)
    float R_measure; // 측정값 노이즈 (기본값: 0.03f)

    float angle; // 필터링된 각도 (State 1)
    float bias;  // 추정된 자이로 바이어스 (State 2)
    float P[2][2]; // 오차 공분산 행렬
} ;

inline float kalman_update(Kalman_t *k, float newAngle, float newRate, float dt) {
    // 1. Predict (예측)
    float rate = newRate - k->bias;
    k->angle += dt * rate;

    k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += k->Q_bias * dt;

    // 2. Update (보정)
    float S = k->P[0][0] + k->R_measure;
    float K[2]; // 칼만 이득
    K[0] = k->P[0][0] / S;
    K[1] = k->P[1][0] / S;

    //float y = newAngle - k->angle; // 측정값 오차

    float y = newAngle - k->angle;
    if (y > 180.0f) y -= 360.0f;
    else if (y < -180.0f) y += 360.0f;

    k->angle += K[0] * y;
    k->bias  += K[1] * y;

    float P00_temp = k->P[0][0];
    float P01_temp = k->P[0][1];

    k->P[0][0] -= K[0] * P00_temp;
    k->P[0][1] -= K[0] * P01_temp;
    k->P[1][0] -= K[1] * P00_temp;
    k->P[1][1] -= K[1] * P01_temp;


    return k->angle;
}

} //namespace KALMAN