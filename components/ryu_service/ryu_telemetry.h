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

class  Telemetry{
 private:
        Telemetry() = default; 
        ~Telemetry() = default;
        static constexpr const char* TAG = "Telemetry";
    public:
        static Telemetry& get_instance() {
            static Telemetry instance; 
            return instance;
        }
        Telemetry(const Telemetry&) = delete;
        Telemetry& operator=(const Telemetry&) = delete;
        Telemetry(Telemetry&&) = delete;
        Telemetry& operator=(Telemetry&&) = delete;

        
        esp_err_t initialize();
        static void telemetry_task(void *pv);
        BaseType_t start_task();


    private:
        TaskHandle_t _task_handle = nullptr;
        bool _initialized = false;
};





}


