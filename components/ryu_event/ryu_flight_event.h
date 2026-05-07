/**
 * @file ryu_flight_event.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-05-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once
#include <esp_event.h>

/**
 * @brief  
 *      1. FLIGHT의 상태를 받아서 어떻게 처리할것인가 ?
 *      2. 이벤트를 받아서 시스템 변수를 바꾼다. 
 */
namespace Event {

    ESP_EVENT_DECLARE_BASE(SYS_MODE_EVENT_BASE);

    enum e_flight_mode {
        MODE_MANUAL,        // 사용자 모드
        MODE_ARM,
        MODE_DISARM,
        MODE_USER_HOLD,     // 사용자 HOLED MODE        
        MODE_ERROR_HOLD,    // 센서 이상 HOLED MODE
        MODE_RTL,           // Return To Launch (배터리 부족 등)
        MODE_LANDING,       // 즉시 착륙
        MODE_FAILSAFE       // 최악의 상황 (모터 정지 등)
    };
  
    typedef struct {
        e_flight_mode new_mode;
        bool  state;             // 모드의 변화가 on(true)/off(false) 
        const char* reason;     // 왜 모드가 바뀌었는지 로그용
    } mode_change_event_t;
    
} // namespace Service
