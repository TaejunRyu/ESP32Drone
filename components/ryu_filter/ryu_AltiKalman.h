#pragma once

#include <cmath>
#include <esp_timer.h>

namespace Filter {

class AltitudeEstimator {
    private:
        AltitudeEstimator() = default;
        ~AltitudeEstimator() = default;
        static constexpr const char* TAG = "AltitudeEstimator";
        
        // 상태 벡터 x = [고도(m), 수직속도(m/s)]
        float z = 0.0f;
        float v = 0.0f;

        // 가속도 바이어스 추정 (수직 가속도 영점 보정용)
        float acc_bias = 0.0f;

        // 필터 게인 (튜닝 파라미터)
        const float K_pos = 0.04f;   // 기압계 고도 보정 게인
        const float K_vel = 0.15f;   // 기압계 기반 속도 보정 게인
        const float K_bias = 0.01f;  // 가속도 바이어스 수정 게인

    public:
        // 복사 및 이동 금지
        AltitudeEstimator(const AltitudeEstimator&) = delete;
        AltitudeEstimator& operator=(const AltitudeEstimator&) = delete;
        
        static AltitudeEstimator& get_instance() {
            static AltitudeEstimator instance;
            return instance;
        }

        /**
         * @brief 400Hz 메인 제어 루프에서 매번 실행하는 가속도 예측 (Prediction)
         * @param raw_az IMU의 Z축 가속도 (m/s^2 단위, 센서 상 기준)
         * @param roll AHRS 칼만 필터에서 계산된 현재 Roll (radian)
         * @param pitch AHRS 칼만 필터에서 계산된 현재 Pitch (radian)
         * @param dt 주기 (400Hz = 0.0025f)
         */
        void predict(float raw_az, float roll, float pitch, float dt) {
            // 1. 센서 좌표계 가속도를 지구 평면 좌표계(수직 가속도)로 변환 (중력 상쇄 포함)
            // 주의: 센서 방향에 따라 부호가 바뀔 수 있습니다. (여기서는 정립 상태 1G = +9.81m/s^2 가정)
            float accel_earth_z = raw_az * std::cos(roll) * std::cos(pitch) - 9.80665f;
            
            // 바이어스 제거
            accel_earth_z -= acc_bias;

            // 2. 400Hz 초고속 적분 (지터 없이 흐르는 기본 상태)
            z += v * dt + 0.5f * accel_earth_z * dt * dt;
            v += accel_earth_z * dt;
        }

        /**
         * @brief 20Hz 기압계 수신 루프(또는 데이터 도착 시점)에서 호출하는 보정 (Update)
         * @param baro_alt 기압계로부터 변환된 현재 절대/상대 고도 (m)
         */
        void update_baro(float baro_alt) {
            // 예측 고도와 기압계 고도의 오차(Innovation) 계산
            float innovation = baro_alt - z;

            // 마할라노비스 거리를 단순화한 게이트 필터 (프로펠러 후류/와류 노이즈 차단)
            // 갑자기 고도가 1.5미터 이상 튀는 기압계 노이즈가 들어오면 업데이트를 차단
            if (std::abs(innovation) > 1.5f) {
                return; 
            }

            // 오차를 상태 변수 전반에 분배 (수직 속도와 가속도 영점까지 보정됨)
            z += K_pos * innovation;
            v += K_vel * innovation;
            acc_bias += K_bias * innovation; // 가속도 드리프트 흡수
        }

        void get_altitude_states(float* altitude, float* climb_rate) {
            *altitude = z;
            *climb_rate = v;
        }

        void reset() {
            z = 0.0f; v = 0.0f; acc_bias = 0.0f;
    }
};

} // namespace Filter