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

namespace Service{

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
        
        Event::e_flight_mode   _flight_mode;     // 현재 비행 모드 => 이건 아직 미정 그냥 qgc와 연계하기 위하여 정의 
        uint8_t         _system_status;      // standby(3), active(4), critical 등
        uint32_t        _system_health;      // 현재 시스템의 상태 ryu_failsafe.h에서 주로 사용
        bool            _is_armed;           // 시동 상태
        bool            _manual_hold_mode;   // flysky controller에서 hold mode 지정
        bool            _error_hold_mode;    // 센서의 오류로 인한 고정 비행
        bool            _gps_ready;          // GPS 수신 준비 완료 (이것이 필요할까?) 
        bool _initialized = false;

};

}//namespace Service

