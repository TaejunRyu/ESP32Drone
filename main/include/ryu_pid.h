#pragma once
/*
 * PID 제어 관련 함수와 변수 선언
 * PID 상수는 paramtable_ryu.h의 파라미터 테이블에서 불러오도록 설계되어 있습니다.
 * PID 계산 함수는 run_pid()로, Yaw 각도 랩어라운드 대응이 포함되어 있습니다.
 * 고도 PID는 기압계 데이터가 업데이트될 때마다 계산하도록 main.cpp에서 호출됩니다.
*/

//여기서  drone_pid_t 는 struct이다만 알려줘도 된다. 
//전체를 알 필요가 없기때문에 이렇게 하면 선언되어지 해더 파일을 include할 필요가 없음 (전방 선언)

namespace Controller{


// PID 구조체
// 프레임워크 전반에서 공유되므로 연속적인 초기화와 링크를 위해 inline 선언합니다.
struct  drone_pid_t{
    float kp;
    float ki;
    float kd;
    float integral;
    float err_prev;  // Outer Loop(각도)용: 이전 오차 저장
    float prev_rate; // Inner Loop(각속도)용: 이전 자이로 값 저장
};


/**
 * @brief 
 *      이 시스템에는 단 1개의  pid만 존재한다.
 */
class PID{
    private:
        PID();
    public:
        // 🌟 복사 생성자와 대입 연산자 비활성화 (싱글톤 복사 방지)
        PID(const PID&) = delete;
        PID& operator=(const PID&) = delete;
        ~PID();

        // 싱글톤 인스턴스 접근 메서드
        // 🌟 get_instance() 메서드 구현
        static PID& get_instance() {
            static PID* instance = new PID(); // 힙에 할당하여 소멸 순서 꼬임 방지
            return *instance;
        }  

        static void  reset_pid(drone_pid_t *p) ;
        static float run_pid_angle(drone_pid_t *p, float tar, float cur, float dt, bool is_yaw);
        static float run_pid_rate(drone_pid_t *p, float target_rate, float current_rate, float dt);
        static void  sync_pid_from_params();
    

        // PID 구조체 초기화 (기본값 0)
        // 제어기 명칭	       역할	    P (Proportional)	                I (Integral)	D (Derivative)	비고
        // Alt Position     (Outer)	    목표 고도 유지	1.0 ~ 2.0	        0.0	            0.0	단위:       (m) -> (m/s) 변환
        // Alt Rate         (Inner)	    상승/하강 속도 제어	50.0 ~ 100.0	 10.0 ~ 20.0	 0.05	        단위: (m/s) -> PWM 변량
        static drone_pid_t pid_alt_pos;     //목표고도유지 
        static drone_pid_t pid_alt_rate;    //상승/하강 속도제어

        // 1. 각도 제어용 (Outer Loop) - P값 위주
        // PID 구조체 초기값 (추천 가이드)
        // 제어기 명칭	        역할	        P (Proportional)	I (Integral)	D (Derivative)	비고
        // Angle (Outer)	    각도 유지	    4.5	                0.0	            0.0	            오직 P값만 사용해도
        static drone_pid_t pid_roll_angle;  //각도유지
        static drone_pid_t pid_pitch_angle;
        static drone_pid_t pid_yaw_angle;

        // 2. 각속도 제어용 (Inner Loop) - 실제 기체 반응 결정
        // 제어기 명칭	        역할	        P (Proportional)	I (Integral)	D (Derivative)	비고
        // Rate (Inner)	        진동/회전 제어	0.15	            0.1	            0.003	        가장 정밀하게 튜닝 필요
        // Yaw Rate	회전        속도 제어	    0.20	            0.05	        0.0	            값은 거의 사용 안 함
        static drone_pid_t pid_roll_rate;   //진동/회전 제어
        static drone_pid_t pid_pitch_rate;
        static drone_pid_t pid_yaw_rate;    // 회전 속도제어
    private:

        bool _initialized;
        static const char* TAG;
};


}