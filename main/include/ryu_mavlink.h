#pragma once
#define MAVLINK_USE_MESSAGE_INFO

#include <driver/uart.h>
#include <lwip/sockets.h>
//#include <common/mavlink.h>
#include <c_library_v2/common/mavlink.h>
#include <c_library_v2/mavlink_get_info.h>
//#include <mavlink_get_info.h>
#include <esp_timer.h>

#include "ryu_pid.h"
#include "ryu_wifi.h"
#include "ryu_buzzer.h"

namespace MAV
{
inline void send_mav_ack(  uint16_t command, 
                    uint8_t result,
                    uint8_t progress, 
                    int32_t result_param2,
                    uint8_t target_sysid,
                    uint8_t target_compid) {
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(
                    SYSTEM_ID, COMPONENT_ID,    // FC의 System/Component ID
                    &msg,
                    command,                    // 응답할 명령 번호 
                    result,                     // 결과 (MAV_RESULT_ACCEPTED)
                    progress, result_param2,    // Progress, Result_param2
                    target_sysid, target_compid // Target System/Component (GCS의 ID)
    );
    WIFI::dispatch_mavlink_msg(&msg);
}

extern void send_status_text(const char* text, uint8_t severity = MAV_SEVERITY_INFO) ;
extern void handle_mavlink_message(mavlink_message_t *msg);

void MAV_CMD_COMPONENT_ARM_DISARM_func(mavlink_message_t *msg, mavlink_command_long_t cmd);
void MAV_CMD_NAV_TAKEOFF_func(mavlink_message_t *msg, mavlink_command_long_t cmd);
void MAV_CMD_DO_SET_HOME_func(mavlink_message_t *msg, mavlink_command_long_t cmd);
void MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_func(mavlink_message_t *msg, mavlink_command_long_t cmd);
void MAV_CMD_REQUEST_MESSAGE_func(mavlink_message_t *msg, mavlink_command_long_t cmd);
void MAV_CMD_PREFLIGHT_CALIBRATION_func(mavlink_message_t *msg, mavlink_command_long_t cmd);
void MAV_CMD_SET_MESSAGE_INTERVAL_func(mavlink_message_t *msg, mavlink_command_long_t cmd);
void MAV_CMD_REQUEST_PROTOCOL_VERSION_func(mavlink_message_t *msg, mavlink_command_long_t cmd);
}