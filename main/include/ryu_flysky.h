#pragma once

#include <driver/uart.h>
#include <driver/mcpwm_cap.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "ryu_config.h"
#include "ryu_common_std.h"
#include "ryu_buzzer.h"
#include "ryu_error_proc.h"
namespace FLYSKY
{
// PPM 신호 캡처 콜백 함수
/*
ppm_values[0]: Aileron (Roll - 좌우 기울기)
ppm_values[1]: Elevator (Pitch - 앞뒤 기울기)
ppm_values[2]: Throttle (스로틀 - 상승/하강)
ppm_values[3]: Rudder (Yaw - 좌우 회전)
ppm_values[4]: Aux 1 (스위치 또는 다이얼) ----> AUTO MODE
ppm_values[5]: Aux 2 (스위치 또는 다이얼)
*/

extern void initialize();
extern void flysky_task(void *pvParameters);

} 