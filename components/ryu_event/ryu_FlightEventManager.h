/**
 * @file ryu_FlightEventManager.h
 * @author your name (you@domain.com)
 * @brief 
 *      1. 하드웨어가 아닌 비행체의 정보변화 이벤트 처리. (FailSafe에서는 시스템의 문제발생시 처리)
 *      2. 비행체의 현재 mode 변화
 *      3.  
 *      4. 
 * @version 0.1
 * @date 2026-05-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma once

#include <esp_err.h>
#include "ryu_flight_event.h"

namespace Event{

class FlightEventManager{
    private:
        FlightEventManager() = default; 
        ~FlightEventManager() = default;
        static constexpr const char* TAG = "FlightEventManager";
    public:
        static FlightEventManager& get_instance() {
            static FlightEventManager instance; 
            return instance;
        }
        FlightEventManager(const FlightEventManager&) = delete;
        FlightEventManager& operator=(const FlightEventManager&) = delete;

        esp_err_t initialize();
        static void event_handler(void *arg, esp_event_base_t base, int32_t id, void *data);

    private:
        

        bool _initialized = false;
};

}//namespace Service

