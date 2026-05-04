/**
 * @file servo.h
 * @author ryutaejun (ryuwillow@gmail.com)
 * @brief   드론의 ESC에 신호를 보내는 작업을 하기 위한 초기화.
 *          
 * @version 0.1
 * @date 2026-03-25
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <array>
#include <driver/gpio.h>
#include <driver/mcpwm_prelude.h> // 신형 MCPWM


namespace Driver
{ 

class Motor{
    private:
        Motor();
    public:
            // 🌟 복사 생성자와 대입 연산자 비활성화 (싱글톤 복사 방지)
        Motor(const Motor&) = delete;
        Motor& operator=(const Motor&) = delete;
        ~Motor();

        // 싱글톤 인스턴스 접근 메서드
        // 🌟 get_instance() 메서드 구현
        static Motor& get_instance() {
            static Motor* instance = new Motor(); // 힙에 할당하여 소멸 순서 꼬임 방지
            return *instance;
        }  
        // FC가 공중 운반물이 있을경우 떨어뜨리는 SERVO MOTOR 
        static inline constexpr  gpio_num_t    SERVO_MOTOR_PIN  = GPIO_NUM_25;
        // 모터 핀 설정 FR, FL, RL, RR
        static inline constexpr int MOTOR_PINS[4] = {GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_32, GPIO_NUM_33}; 

        esp_err_t initialize();
        esp_err_t stop_all_motors();
        void set_drop_angle(int angle);
        void update_compare_value(std::array<float,4> values);

    private:
        mcpwm_cmpr_handle_t _comparators[4] {};     // 모터 듀티 제어용
        mcpwm_timer_handle_t _timer = nullptr;             // mcpwm handle
        mcpwm_gen_handle_t  _servo_gen = nullptr;        
        mcpwm_cmpr_handle_t _servo_comparator = nullptr;   // 서보 각도 조절용

        bool _initialized = false;
        static const char* TAG;

};

}