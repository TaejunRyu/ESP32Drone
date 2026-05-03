/*
Attitude (5Hz): 4번에 한 번 (0, 4, 8, 12, 16번 슬롯)
Global Position (2Hz): 10번에 한 번 (2, 12번 슬롯)
SYS Status (1Hz): 20번에 한 번 (6번 슬롯)
Heartbeat (1Hz): 20번에 한 번 (10번 슬롯)
*/
#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>


namespace TELEM
{

typedef struct {
    uint8_t data[290];
    size_t len;
} esp_now_data_t;

extern QueueHandle_t mavlink_rx_queue;

extern void initialize();
extern void telemetry_task(void *pv);
}


