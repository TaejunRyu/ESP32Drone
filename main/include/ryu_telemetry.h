/*
Attitude (5Hz): 4번에 한 번 (0, 4, 8, 12, 16번 슬롯)
Global Position (2Hz): 10번에 한 번 (2, 12번 슬롯)
SYS Status (1Hz): 20번에 한 번 (6번 슬롯)
Heartbeat (1Hz): 20번에 한 번 (10번 슬롯)
*/
#pragma once

#include <esp_timer.h>
#include <driver/uart.h>
#include <driver/mcpwm_prelude.h> // 신형 MCPWM
#include <esp_log.h>
#include <lwip/sockets.h>
//#include <common/mavlink.h>
#include <c_library_v2/common/mavlink.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

#include "ryu_battery.h"
#include "ryu_mavlink.h"
#include "ryu_buzzer.h"
#include "ryu_paramtable.h"
#include "ryu_pid.h"
#include "ryu_config.h"
#include "ryu_gps.h"
#include "ryu_wifi.h"
#include "ryu_flight.h"

namespace TELEM
{

typedef struct {
    uint8_t data[ESP_NOW_MAX_LEN];
    size_t len;
} esp_now_data_t;

extern QueueHandle_t mavlink_rx_queue;

extern void initialize();
extern void telemetry_task(void *pv);
}


