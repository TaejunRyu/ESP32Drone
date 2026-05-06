#pragma once
#include <math.h>
#include <algorithm>
#include <esp_err.h>
namespace Service{

class Mahony{
 private:
        Mahony() = default; 
        ~Mahony() = default;
        static constexpr const char* TAG = "Mahony";
    public:
        static Mahony& get_instance() {
            static Mahony instance; 
            return instance;
        }
        Mahony(const Mahony&) = delete;
        Mahony& operator=(const Mahony&) = delete;
        Mahony(Mahony&&) = delete;
        Mahony& operator=(Mahony&&) = delete;

        const float INTEGRAL_MAX = 0.5f;


        float q0, q1, q2, q3; // 쿼터니언
        float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // 적분 오차
        void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
        void calibrate_mahony_initial_attitude(float ax, float ay, float az, float mx, float my, float mz);
        void reset_mahony_integral(void);

        esp_err_t initialize();
        bool is_initialized(){return _initialized;};

    private:
        //초기 안정화를 빠르게하기 위한 변수
        bool is_booting = true;
        uint64_t boot_start_time; // booting time
    
        bool _initialized = false;
};



}