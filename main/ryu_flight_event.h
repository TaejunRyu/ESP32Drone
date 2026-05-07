#pragma once
#include <esp_event.h>

/**
 * @brief  
 *      1. FLIGHT의 상태를 받아서 어떻게 처리할것인가 ?
 * 
 */
namespace EVENT {

    ESP_EVENT_DECLARE_BASE(SYS_MODE_EVENT_BASE);

     enum e_flight_mode {
        MODE_MANUAL,
        MODE_HOLD,     // 센서 이상 시 자동 진입
        MODE_RTL,      // Return To Launch (배터리 부족 등)
        MODE_LANDING,  // 즉시 착륙
        MODE_FAILSAFE  // 최악의 상황 (모터 정지 등)
    };
  
    typedef struct {
        e_flight_mode new_mode;
        const char* reason; // 왜 모드가 바뀌었는지 로그용
    } mode_change_event_t;
    
} // namespace Service
