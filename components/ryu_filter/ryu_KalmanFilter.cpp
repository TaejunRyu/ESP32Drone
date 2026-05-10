#include "ryu_KalmanFilter.h"

namespace Filter {

KalmanFilter::KalmanFilter() {
    reset();
}

void KalmanFilter::reset() {
    x[0] = 1.0f; x[1] = 0.0f; x[2] = 0.0f; x[3] = 0.0f;
    x[4] = 0.0f; x[5] = 0.0f; x[6] = 0.0f;
    
    for(int i=0; i<7; i++) {
        for(int j=0; j<7; j++) P[i][j] = 0.0f;
        P[i][i] = 0.1f; // 초기 불확실성 설정
    }
}

void KalmanFilter::update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    predict(gx, gy, gz, dt);
    update_accel(ax, ay, az);
    normalize_quat();
}

void KalmanFilter::predict(float gx, float gy, float gz, float dt) {
    // 1. 자이로 바이어스 제거
    float wx = gx - x[4];
    float wy = gy - x[5];
    float wz = gz - x[6];

    // 2. 쿼터니언 미분 방정식 (상태 예측)
    float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];
    x[0] += 0.5f * (-q1 * wx - q2 * wy - q3 * wz) * dt;
    x[1] += 0.5f * ( q0 * wx + q2 * wz - q3 * wy) * dt;
    x[2] += 0.5f * ( q0 * wy - q1 * wz + q3 * wx) * dt;
    x[3] += 0.5f * ( q0 * wz + q1 * wy - q2 * wx) * dt;

    // 3. 공분산 업데이트 (P = FPF' + Q)
    // ESP32-S3의 성능을 활용한 대각항 간략화 업데이트
    for(int i=0; i<4; i++) P[i][i] += Q_quat * dt;
    for(int i=4; i<7; i++) P[i][i] += Q_bias * dt;

    // P값이 너무 커지는 것을 방지 (Safety Clamp)
    for(int i=0; i<7; i++) {
        if (P[i][i] > 1.0f) P[i][i] = 1.0f;
    }
}


void KalmanFilter::update_accel(float ax, float ay, float az) {
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.5f || norm > 1.5f) return; // 급격한 가속 중에는 보정 안 함
    ax /= norm; ay /= norm; az /= norm;

    // 중력 예상 방향 (Estimated Gravity)
    float vx = 2.0f * (x[1]*x[3] - x[0]*x[2]);
    float vy = 2.0f * (x[0]*x[1] + x[2]*x[3]);
    //float vz = x[0]*x[0] - x[1]*x[1] - x[2]*x[2] + x[3]*x[3];
    float vz = -x[0]*x[0] + x[1]*x[1] + x[2]*x[2] - x[3]*x[3];       // NED 표준으로 교체.

    // Innovation (측정 오차)
    float ex = ax - vx;
    float ey = ay - vy;
    float ez = az - vz;

    // 칼만 이득(K)을 적용한 상태 보정 
    // S3의 FPU 성능을 고려하여 가중치를 통해 상태값 업데이트
    float K = 0.01f; // 고정 게인 사용 시 안정적
    x[0] += K * ( -x[2]*ex + x[1]*ey );
    x[1] += K * (  x[3]*ex + x[0]*ey - 2.0f*x[1]*ez );
    x[2] += K * ( -x[0]*ex + x[3]*ey - 2.0f*x[2]*ez );
    x[3] += K * (  x[1]*ex + x[2]*ey );
    
    // 바이어스 보정 
    x[4] -= K * 0.01f * ex;
    x[5] -= K * 0.01f * ey;
    x[6] -= K * 0.01f * ez;
}

void KalmanFilter::normalize_quat() {
    float n = sqrtf(x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3]);
    if (n > 0.0f) {
        float recip = 1.0f / n;
        x[0] *= recip; x[1] *= recip; x[2] *= recip; x[3] *= recip;
    }
}

void KalmanFilter::get_euler(float* roll, float* pitch, float* yaw) {
    *roll  = atan2f(2.0f * (x[0] * x[1] + x[2] * x[3]), 1.0f - 2.0f * (x[1] * x[1] + x[2] * x[2])) * 57.29578f;
    *pitch = asinf(std::clamp(2.0f * (x[0] * x[2] - x[3] * x[1]), -1.0f, 1.0f)) * 57.29578f;
    *yaw   = atan2f(2.0f * (x[0] * x[3] + x[1] * x[2]), 1.0f - 2.0f * (x[2] * x[2] + x[3] * x[3])) * 57.29578f;
}


void KalmanFilter::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
    predict(gx, gy, gz, dt);      // 1. 자이로 기반 예측
    update_accel(ax, ay, az);     // 2. 가속도계 기반 Roll/Pitch 보정
    update_mag(mx, my, mz);       // 3. 지자계 기반 Yaw 보정 (추가됨)
    normalize_quat();             // 4. 정규화
}

void KalmanFilter::update_mag(float mx, float my, float mz) {
    float norm = sqrtf(mx*mx + my*my + mz*mz);
    if (norm < 0.1f) return;
    mx /= norm; my /= norm; mz /= norm;

    float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];

    // 1. 지구 자기장 수평(bx)/수직(bz) 성분 측정 (Mahony와 동일)
    float hx = mx * (q0*q0 + q1*q1 - q2*q2 - q3*q3) + my * 2.0f * (q1*q2 - q0*q3) + mz * 2.0f * (q1*q3 + q0*q2);
    float hy = mx * 2.0f * (q1*q2 + q0*q3) + my * (q0*q0 - q1*q1 + q2*q2 - q3*q3) + mz * 2.0f * (q2*q3 - q0*q1);
    float bx = sqrtf(hx * hx + hy * hy);
    float bz = mx * 2.0f * (q1*q3 - q0*q2) + my * 2.0f * (q2*q3 + q0*q1) + mz * (q0*q0 - q1*q1 - q2*q2 + q3*q3);

    // 2. 자기장 예상 방향 (기체 좌표계로 변환)
    float wx = bx * (q0*q0 + q1*q1 - q2*q2 - q3*q3) + bz * 2.0f * (q1*q3 - q0*q2);
    float wy = bx * 2.0f * (q1*q2 - q0*q3) + bz * 2.0f * (q0*q1 + q2*q3);
    float wz = bx * 2.0f * (q0*q2 + q1*q3) + bz * (q0*q0 - q1*q1 - q2*q2 + q3*q3);

    // 3. 외적(Cross Product)을 이용한 Yaw 오차 추출 (Mahony 방식)
    // 이 방식이 축 반전에 훨씬 강하고 안정적입니다.
    float ex_mag = (my * wz - mz * wy);
    float ey_mag = (mz * wx - mx * wz);
    float ez_mag = (mx * wy - my * wx);

    // 4. 칼만 업데이트 (상태 보정)
    float K = 0.01f; // S3 400Hz 기준 적절한 게인
    x[0] += K * (-q1 * ex_mag - q2 * ey_mag - q3 * ez_mag);
    x[1] += K * ( q0 * ex_mag + q2 * ez_mag - q3 * ey_mag);
    x[2] += K * ( q0 * ey_mag - q1 * ez_mag + q3 * ex_mag);
    x[3] += K * ( q0 * ez_mag + q1 * ey_mag - q2 * ex_mag);
    
    // 바이어스(bx, by, bz)도 느리게 학습
    x[4] -= K * 0.1f * ex_mag;
    x[5] -= K * 0.1f * ey_mag;
    x[6] -= K * 0.1f * ez_mag;
}



} // namespace Service



// #include "ryu_KalmanFilter.h"
// #include "esp_log.h"

// namespace Service {

// KalmanFilter::KalmanFilter() {
//     reset();
//     // dsp 라이브러리 초기화 확인 (필요시)
//     esp_err_t ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
// }

// void KalmanFilter::reset() {
//     std::fill_n(x, 7, 0.0f);
//     x[0] = 1.0f; // Identity Quaternion
    
//     std::fill_n(P, 49, 0.0f);
//     for(int i=0; i<7; i++) P[i*7 + i] = 0.1f; // 초기 불확실성

//     std::fill_n(Q, 49, 0.0f);
//     for(int i=0; i<3; i++) Q[i*7 + i] = 0.001f; // Quat noise
//     for(int i=4; i<7; i++) Q[i*7 + i] = 0.00001f; // Bias noise
// }

// void KalmanFilter::predict(float gx, float gy, float gz, float dt) {
//     float wx = gx - x[4];
//     float wy = gy - x[5];
//     float wz = gz - x[6];

//     // 1. 상태 예측 (Runge-Kutta 1st)
//     float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];
//     x[0] += 0.5f * (-q1 * wx - q2 * wy - q3 * wz) * dt;
//     x[1] += 0.5f * ( q0 * wx + q2 * wz - q3 * wy) * dt;
//     x[2] += 0.5f * ( q0 * wy - q1 * wz + q3 * wx) * dt;
//     x[3] += 0.5f * ( q0 * wz + q1 * wy - q2 * wx) * dt;

//     // 2. F 행렬 (Jacobian) 구성
//     std::fill_n(F, 49, 0.0f);
//     for(int i=0; i<7; i++) F[i*7+i] = 1.0f;
    
//     // 쿼터니언 미분 기반 F 행렬 채우기 (간략화된 형태)
//     F[1] = -0.5f*wx*dt; F[2] = -0.5f*wy*dt; F[3] = -0.5f*wz*dt;
//     F[7] =  0.5f*wx*dt; F[10] = 0.5f*wz*dt; F[11] = -0.5f*wy*dt;
//     // ... (이하 생략된 항들은 S3의 SIMD mmult 성능으로 커버 가능)

//     // 3. P = FPF' + Q (esp-dsp 가속)
//     float tmp[49] __attribute__((aligned(16)));
//     float FT[49] __attribute__((aligned(16)));
    
//     for(int i=0; i<7; i++) for(int j=0; j<7; j++) FT[i*7+j] = F[j*7+i];

//     dsps_mmult_f32(F, P, tmp, 7, 7, 7);
//     dsps_mmult_f32(tmp, FT, P, 7, 7, 7);
//     dsps_add_f32(P, Q, P, 49);
// }

// void KalmanFilter::update_accel(float ax, float ay, float az) {
//     float norm = sqrtf(ax*ax + ay*ay + az*az);
//     if (norm < 0.5f || norm > 1.5f) return;
//     ax /= norm; ay /= norm; az /= norm;

//     float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];
//     float vx = 2.0f * (q1*q3 - q0*q2);
//     float vy = 2.0f * (q0*q1 + q2*q3);
//     float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

//     // Jacobian H (3x7)
//     float H[21] = {
//         -2.0f*q2,  2.0f*q3, -2.0f*q0,  2.0f*q1, 0, 0, 0,
//          2.0f*q1,  2.0f*q0,  2.0f*q3,  2.0f*q2, 0, 0, 0,
//          2.0f*q0, -2.0f*q1, -2.0f*q2,  2.0f*q3, 0, 0, 0
//     };

//     // Kalman Gain K = PH' / (HPH' + R) 계산 (esp-dsp 활용)
//     // 실제 구현에서는 3x3 역행렬 연산이 포함됩니다.
//     float R_acc = 0.01f;
//     float innov[3] = { ax - vx, ay - vy, az - vz };
    
//     // 가중치 업데이트 (S3 가속을 위해 단순화된 내적 활용 가능)
//     float K = 0.01f; 
//     for(int i=0; i<7; i++) {
//         float h_row_dot = H[0*7+i]*innov[0] + H[1*7+i]*innov[1] + H[2*7+i]*innov[2];
//         x[i] += K * h_row_dot;
//     }
// }

// void KalmanFilter::get_euler(float* roll, float* pitch, float* yaw) {
//     *roll  = atan2f(2.0f * (x[0]*x[1] + x[2]*x[3]), 1.0f - 2.0f * (x[1]*x[1] + x[2]*x[2])) * 57.29578f;
//     *pitch = asinf(std::clamp(2.0f * (x[0]*x[2] - x[3]*x[1]), -1.0f, 1.0f)) * 57.29578f;
//     *yaw   = atan2f(2.0f * (x[0]*x[3] + x[1]*x[2]), 1.0f - 2.0f * (x[2]*x[2] + x[3]*x[3])) * 57.29578f;
// }

// // ... update_mag 및 나머지 함수는 Mahony 외적 로직과 결합하여 구현 ...
// void KalmanFilter::update_mag(float mx, float my, float mz) {
//     float norm = sqrtf(mx*mx + my*my + mz*mz);
//     if (norm < 0.1f) return;
//     mx /= norm; my /= norm; mz /= norm;

//     float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];

//     // 1. 지구 자기장 수평(bx)/수직(bz) 성분 측정 (Mahony와 동일)
//     float hx = mx * (q0*q0 + q1*q1 - q2*q2 - q3*q3) + my * 2.0f * (q1*q2 - q0*q3) + mz * 2.0f * (q1*q3 + q0*q2);
//     float hy = mx * 2.0f * (q1*q2 + q0*q3) + my * (q0*q0 - q1*q1 + q2*q2 - q3*q3) + mz * 2.0f * (q2*q3 - q0*q1);
//     float bx = sqrtf(hx * hx + hy * hy);
//     float bz = mx * 2.0f * (q1*q3 - q0*q2) + my * 2.0f * (q2*q3 + q0*q1) + mz * (q0*q0 - q1*q1 - q2*q2 + q3*q3);

//     // 2. 자기장 예상 방향 (기체 좌표계로 변환)
//     float wx = bx * (q0*q0 + q1*q1 - q2*q2 - q3*q3) + bz * 2.0f * (q1*q3 - q0*q2);
//     float wy = bx * 2.0f * (q1*q2 - q0*q3) + bz * 2.0f * (q0*q1 + q2*q3);
//     float wz = bx * 2.0f * (q0*q2 + q1*q3) + bz * (q0*q0 - q1*q1 - q2*q2 + q3*q3);

//     // 3. 외적(Cross Product)을 이용한 Yaw 오차 추출 (Mahony 방식)
//     // 이 방식이 축 반전에 훨씬 강하고 안정적입니다.
//     float ex_mag = (my * wz - mz * wy);
//     float ey_mag = (mz * wx - mx * wz);
//     float ez_mag = (mx * wy - my * wx);

//     // 4. 칼만 업데이트 (상태 보정)
//     float K = 0.01f; // S3 400Hz 기준 적절한 게인
//     x[0] += K * (-q1 * ex_mag - q2 * ey_mag - q3 * ez_mag);
//     x[1] += K * ( q0 * ex_mag + q2 * ez_mag - q3 * ey_mag);
//     x[2] += K * ( q0 * ey_mag - q1 * ez_mag + q3 * ex_mag);
//     x[3] += K * ( q0 * ez_mag + q1 * ey_mag - q2 * ex_mag);
    
//     // 바이어스(bx, by, bz)도 느리게 학습
//     x[4] -= K * 0.1f * ex_mag;
//     x[5] -= K * 0.1f * ey_mag;
//     x[6] -= K * 0.1f * ez_mag;
// }


// } // namespace Service
