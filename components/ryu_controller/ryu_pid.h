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

        void  reset_pid(drone_pid_t *p) ;
        float run_pid_angle(drone_pid_t *p, float tar, float cur, float dt, bool is_yaw);
        float run_pid_rate(drone_pid_t *p, float target_rate, float current_rate, float dt);
        void  sync_pid_from_params();
        void  initialize();

        drone_pid_t pid_alt_pos {};     //목표고도유지 
        drone_pid_t pid_alt_rate {};    //상승/하강 속도제어

        drone_pid_t pid_roll_angle {};  //각도유지
        drone_pid_t pid_pitch_angle {};
        drone_pid_t pid_yaw_angle {};

        drone_pid_t pid_roll_rate {};    //진동회전 제어
        drone_pid_t pid_pitch_rate {};
        drone_pid_t pid_yaw_rate {};    // 회전 속도제어
    private:

        bool _initialized = false;
        static const char* TAG;
};


}