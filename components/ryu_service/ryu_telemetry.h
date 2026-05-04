/*
Attitude (5Hz): 4번에 한 번 (0, 4, 8, 12, 16번 슬롯)
Global Position (2Hz): 10번에 한 번 (2, 12번 슬롯)
SYS Status (1Hz): 20번에 한 번 (6번 슬롯)
Heartbeat (1Hz): 20번에 한 번 (10번 슬롯)
*/
#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>


namespace Service
{

// extern void initialize();
// extern void telemetry_task(void *pv);

class  Telemetry{
    private:
        Telemetry();
    public:
        // 🌟 복사 생성자와 대입 연산자 비활성화 (싱글톤 복사 방지)
        Telemetry(const Telemetry&) = delete;
        Telemetry& operator=(const Telemetry&) = delete;
        ~Telemetry();

        // 싱글톤 인스턴스 접근 메서드
        // 🌟 get_instance() 메서드 구현
        static Telemetry& get_instance() {
            static Telemetry* instance = new Telemetry(); // 힙에 할당하여 소멸 순서 꼬임 방지
            return *instance;
        }  
        
        void initialize();
        static void telemetry_task(void *pv);
        void start_task();


    private:
        bool _initialized = false;
        static const char* TAG;
};





}


