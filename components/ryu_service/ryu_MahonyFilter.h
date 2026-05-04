#pragma once
#include <math.h>
#include <algorithm>

namespace Service{

class Mahony{
    private:
        Mahony();
    public:
            // 🌟 복사 생성자와 대입 연산자 비활성화 (싱글톤 복사 방지)
        Mahony(const Mahony&) = delete;
        Mahony& operator=(const Mahony&) = delete;
        ~Mahony();

        // 싱글톤 인스턴스 접근 메서드
        // 🌟 get_instance() 메서드 구현
        static Mahony& get_instance() {
            static Mahony* instance = new Mahony(); // 힙에 할당하여 소멸 순서 꼬임 방지
            return *instance;
        }  
        const float INTEGRAL_MAX = 0.5f;


        float q0, q1, q2, q3; // 쿼터니언
        float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // 적분 오차
        void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
        void calibrate_mahony_initial_attitude(float ax, float ay, float az, float mx, float my, float mz);
        void reset_mahony_integral(void);

        void initialize();

    private:
        //초기 안정화를 빠르게하기 위한 변수
        bool is_booting = true;
        uint64_t boot_start_time; // booting time
    
        bool _initialized = false;
        static const char* TAG;
};



}