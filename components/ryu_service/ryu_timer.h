#pragma once

#include <functional>
#include <freertos/FreeRTOS.h>  // FreeRTOS 기본 설정 및 정의
#include <freertos/timers.h>    // 소프트웨어 타이머 API 전용 헤더

namespace Service{ 

class Timer{
    private:
        Timer();
    public:
            // 🌟 복사 생성자와 대입 연산자 비활성화 (싱글톤 복사 방지)
        Timer(const Timer&) = delete;
        Timer& operator=(const Timer&) = delete;
        ~Timer();

        // 싱글톤 인스턴스 접근 메서드
        // 🌟 get_instance() 메서드 구현
        static Timer& get_instance() {
            static Timer* instance = new Timer(); // 힙에 할당하여 소멸 순서 꼬임 방지
            return *instance;
        }  
        esp_err_t intiallize();
        
        // 콜백 설정
        void set_timer_callback(std::function<void()> callback);

        // 다른 class의 method을 연결해서 실행할때 사용.
        static void timer_callback(TimerHandle_t xTimer);

        esp_err_t Start();
        bool is_running(){return _running;};
        void Stop();
        void Restart();
        void Delete();
        void Change_Period(uint16_t n);
    private:
        std::function<void()> _timer_callback;
        
        TimerHandle_t _timer_handle = nullptr;
        bool _initialized = false;
        bool _running = false;
        static const char* TAG;

};




} //namespace Service