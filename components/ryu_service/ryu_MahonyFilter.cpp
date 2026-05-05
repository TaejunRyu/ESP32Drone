#include "ryu_MahonyFilter.h"

#include <esp_log.h>
#include <esp_timer.h>
#include "ryu_config.h"


namespace Service{

const char* Mahony::TAG="Mahony";

Mahony::Mahony(){
    ESP_LOGI(TAG,"Initializing Service...");
}

Mahony::~Mahony()
{
}

// ========== PID 파라미터 (현재 드론 튜닝값) ==========
// Kp: P 게인 - 반응 속도 결정 (높으면 빠르나 진동 증가)
//   권장 범위: 1.0 ~ 3.0
//   - 1.0 ~ 1.5: 느리지만 안정적 (초보용)
//   - 2.0 ~ 2.5: 중간 (일반적 추천)
//   - 2.5 ~ 3.0: 빠르지만 진동 가능
//const float Kp = 2.0f; 
// Ki: I 게인 - 지자계/중력 축적 오차 보정
//   권장 범위: 0.005 ~ 0.02
//   - 너무 크면: 적분 windup으로 불안정 (반드시 클램핑 필요)
//   - 너무 작으면: Yaw drift 미보정
//const float Ki = 0.005f; // ← 현재 값 (Yaw drift 많으면 증가 권장)

// Integral Windup 방지용 클램핑 범위
// 이 값이 커질수록 적분값이 더 누적될 수 있음 (보통 0.3 ~ 0.5 권장)
void Mahony::MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt)
{    
    float Kp = 2.0f; // 평상시 게인
    float Ki = 0.005f;
    if (is_booting){    
        // 부팅 후 3초 동안은 강하게 수렴(Fast Convergence)
        if (esp_timer_get_time() - boot_start_time < 3'000'000) {
            Kp = 20.0f; 
            Ki = 0.0f; // 초기 정렬 시 적분항은 0으로 두는 것이 오버슈트 방지에 좋습니다.
        }else{
            Kp = 2.0f; 
            Ki = 0.005f;
            is_booting = false; 
        }
    }
    float recipNorm;
    float ex=0.0f, ey=0.0f, ez=0.0f;
    float bx = 0.0f, bz = 0.0f; // 초기화 필수
    float wx = 0.0f, wy = 0.0f, wz = 0.0f; // 초기화 필수

    // 1. 가속도 정규화
    float a_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (a_norm > 0.0f) {
        recipNorm = 1.0f / a_norm;
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
    }
    // 2. 지자계 정규화
    float m_norm = sqrtf(mx * mx + my * my + mz * mz);
    if (m_norm > 0.0f) {
        recipNorm = 1.0f / m_norm;
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;
        
        //Tilt-Compensated Yaw 초기화 (Xh, Yh)드론이 지면에서 약간 기울어져 있더라도 
        //가속도(Roll, Pitch)를 기반으로 지자기 데이터를 수평면으로 투영하여 정확한 절대 북쪽(Yaw)을 바라보며 
        //부팅되도록 정석대로 구현.
        // 지구 자기장 예측 (기체 좌표계 -> 지구 좌표계)
        float hx = mx * (q0*q0 + q1*q1 - q2*q2 - q3*q3) + my * 2.0f * (q1*q2 - q0*q3) + mz * 2.0f * (q1*q3 + q0*q2);
        float hy = mx * 2.0f * (q1*q2 + q0*q3) + my * (q0*q0 - q1*q1 + q2*q2 - q3*q3) + mz * 2.0f * (q2*q3 - q0*q1);

        //지구 자기장 수평 성분이 부분이 핵심입니다.
        bx = sqrtf(hx * hx + hy * hy); 
        bz = mx * 2.0f * (q1*q3 - q0*q2) + my * 2.0f * (q2*q3 + q0*q1) + mz * (q0*q0 - q1*q1 - q2*q2 + q3*q3); // 수직 성분

        //자기장 예상 방향 (지구 좌표계 -> 기체 좌표계)
        wx = bx * (q0*q0 + q1*q1 - q2*q2 - q3*q3) + bz * 2.0f * (q1*q3 - q0*q2);
        wy = bx * 2.0f * (q1*q2 - q0*q3) + bz * 2.0f * (q0*q1 + q2*q3);
        wz = bx * 2.0f * (q0*q2 + q1*q3) + bz * (q0*q0 - q1*q1 - q2*q2 + q3*q3);
    }
    // 3. 중력 예상 방향
    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 4. 오차 계산 (★ 정규화된 mx, my, mz가 쓰이므로 가속도 오차와 밸런스가 맞습니다)
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    // 5. PI 제어 및 쿼터니언 업데이트
    // [중요] Integral Windup 방지: 적분값 클램핑
    // 지속적인 오류 상황에서 적분항이 폭발적으로 증가하는 것을 방지합니다.
    integralFBx = std::clamp(integralFBx + ex * Ki * dt, -INTEGRAL_MAX, INTEGRAL_MAX);
    integralFBy = std::clamp(integralFBy + ey * Ki * dt, -INTEGRAL_MAX, INTEGRAL_MAX);
    integralFBz = std::clamp(integralFBz + ez * Ki * dt, -INTEGRAL_MAX, INTEGRAL_MAX);
    

    // 5. PI 제어 적용된 각속도 계산 (보정된 각속도)
    float gx_corr = gx + (Kp * ex + integralFBx);
    float gy_corr = gy + (Kp * ey + integralFBy);
    float gz_corr = gz + (Kp * ez + integralFBz);

   // 6. 보정된 각속도로 쿼터니언 업데이트 (수식 최적화 및 정석 적용)
    float q0_next = q0 + (-q1 * gx_corr - q2 * gy_corr - q3 * gz_corr) * (0.5f * dt);
    float q1_next = q1 + ( q0 * gx_corr + q2 * gz_corr - q3 * gy_corr) * (0.5f * dt);
    float q2_next = q2 + ( q0 * gy_corr - q1 * gz_corr + q3 * gx_corr) * (0.5f * dt);
    float q3_next = q3 + ( q0 * gz_corr + q1 * gy_corr - q2 * gx_corr) * (0.5f * dt);

    q0 = q0_next; q1 = q1_next; q2 = q2_next; q3 = q3_next;

    // 정규화
    recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;

}

// ========== 캘리브레이션 함수 ==========
/**
 * @brief 드론이 수평(LEVEL) 상태일 때 초기 롤/피치 설정
 * 
 * 사용 방법:
 *   1. 드론을 완전히 수평으로 놓음
 *   2. 비행 코드 시작 전에 한 번만 호출
 *   3. 이후 쿼터니언이 올바르게 초기화됨
 * 
 * @param ax, ay, az 정규화된 가속도계 값 (또는 평균값)
 */
void Mahony::calibrate_mahony_initial_attitude(float ax, float ay, float az, float mx, float my, float mz)
{
    // 1. 가속도계 정규화
    float a_norm = sqrtf(ax*ax + ay*ay + az*az);
    if (a_norm < 0.001f) return; 
    ax /= a_norm; ay /= a_norm; az /= a_norm;

    // 2. 가속도 데이터를 이용해 초기 Roll, Pitch 직접 계산 (Radian)
    // 수평 상태라면 roll, pitch는 0에 매우 가깝게 나옴
    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

      // 3. 지자계 데이터를 이용해 초기 Yaw(Heading) 계산 (Tilt-compensated)
    // mx, my, mz는 반드시 오프셋 보정(Hard/Soft Iron)이 완료된 값이어야 합니다.
    float cosR = cosf(roll);
    float sinR = sinf(roll);
    float cosP = cosf(pitch);
    float sinP = sinf(pitch);

    //Tilt-Compensated Yaw 초기화 (Xh, Yh)드론이 지면에서 약간 기울어져 있더라도 
    //가속도(Roll, Pitch)를 기반으로 지자기 데이터를 수평면으로 투영하여 정확한 절대 북쪽(Yaw)을 바라보며 
    //부팅되도록 정석대로 구현.
    float Xh = mx * cosP + my * sinP * sinR + mz * sinP * cosR;
    float Yh = my * cosR - mz * sinR;

    float yaw = atan2f(-Yh, Xh);   

    // 3. 오일러 각을 쿼터니언으로 즉시 변환 (Direct Assignment)
    float cr = cosf(roll * 0.5f);  float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f); float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);   float sy = sinf(yaw * 0.5f);

    q0 = cr * cp * cy + sr * sp * sy;
    q1 = sr * cp * cy - cr * sp * sy;
    q2 = cr * sp * cy + sr * cp * sy;
    q3 = cr * cp * sy - sr * sp * cy;

    // 4. 적분항 및 오차 변수 완벽 초기화 (매우 중요)
    integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
    // Mahony 내부에서 사용하는 이전 오차값들도 있다면 0으로 리셋하세요.    
}

void Mahony::reset_mahony_integral(void)
{
    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;

}

esp_err_t Mahony::initialize()
{   
    if (_initialized) return ESP_OK;
    q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 쿼터니언
    integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // 적분 오차
    boot_start_time = esp_timer_get_time();// esp_log_timestamp();
    _initialized = true;
    ESP_LOGI(TAG,"Initialized successfully.");
    return ESP_OK;
}
}