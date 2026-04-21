#pragma once

#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ryu_config.h"

namespace BUZZ{
extern void initialize();
extern void play_tone(uint32_t freq_hz, uint32_t duration_ms);
extern void sound_success();
extern void sound_error();
extern void sound_low_battery();
extern void sound_mission_complete();
extern void sound_system_start();
extern void sound_emergency();
extern void sound_click();
extern void sound_system_off();
extern void sound_processing();
extern void sound_surprise();
extern void sound_sad();
extern void sound_question();
extern void sound_proximity_alert();
extern void sound_scanning();
extern void sound_connected();
extern void sound_disconnected();
}