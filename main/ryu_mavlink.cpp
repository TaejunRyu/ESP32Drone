#include "ryu_mavlink.h"

#include <cmath>   // float, double용 std::abs
#include <driver/uart.h>
#include <lwip/sockets.h>
#include <esp_timer.h>

#include "ryu_ParamTable.h"
#include "ryu_pid.h"
#include "ryu_wifi.h"
#include "ryu_buzzer.h"

/**
 * @brief MAVLink 메시지 전송 함수
 * 
 * @param msg 
 * @return * void 
 */

namespace MAV{

void send_mav_ack(  uint16_t command, 
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

// QGC(Mission Planner 포함)는 STATUSTEXT 메시지가 수신되면 설정에 따라 이를 화면에 표시하고 영문/국문 음성으로 읽어줍니다.
// 사용 예시
// 1. 시스템 시작 시   : send_status_text("Bridge Started!", MAV_SEVERITY_INFO);
// 2. 드론 연결 시     : send_status_text("Drone Connected", MAV_SEVERITY_NOTICE);
// 3. 심각한 에러 발생 시: send_status_text("Queue Full! Data Loss", MAV_SEVERITY_ALERT);

void send_status_text(const char* text, uint8_t severity) {
    mavlink_message_t msg;
    char buf[50] = {};
    strncpy(buf, text, sizeof(buf) - 1);
    // severity: MAV_SEVERITY_INFO (6), MAV_SEVERITY_WARNING (4), MAV_SEVERITY_CRITICAL (2) 등
    mavlink_msg_statustext_pack(
        SYSTEM_ID,
        COMPONENT_ID,
        &msg,
        severity,
        buf,0,0
    );
    WIFI::dispatch_mavlink_msg(&msg);
}

/**
 * @brief telemetry에서 데이터를 받으면 분석하여 각 msg/command별로 나누어서 실행한다.
 * 
 * @param msg 
 */
void handle_mavlink_message(mavlink_message_t *msg) {
    const mavlink_message_info_t *ret_msg= mavlink_get_message_info_by_id(msg->msgid);    
    
    switch (msg->msgid) {      
        case MAVLINK_MSG_ID_SYSTEM_TIME:{
            mavlink_message_t ret_msg;
            mavlink_msg_system_time_pack(SYSTEM_ID, COMPONENT_ID,&ret_msg,    // 보통 1 (Autopilot)                
                0,                                                                      // Param 1: Unix time (us)
                (uint32_t)(esp_timer_get_time() / 1000)                                 // Param 2: Boot time (ms)
            );
            WIFI::dispatch_mavlink_msg(&ret_msg);
            break;
        }
        // 1. QGC가 "네가 가진 파라미터 다 내놔"라고 할 때 (연결 초기)
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: 
        {               
            mavlink_param_request_list_t req;
            mavlink_msg_param_request_list_decode(msg,&req);
            
            if (req.target_system != SYSTEM_ID || 
                (req.target_component != COMPONENT_ID && req.target_component != 0)) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            // 이전 전송테이터가 쌓이지 않도록 여유를 준다.
            for (int i = 0 ; auto &par : PARAM::params ){
                float *param_ptr =  reinterpret_cast<float*>(&PARAM::values);
                // 전송부 코드 예시
                float val_to_send;
                if (par.type == 6) { // INT32
                    int32_t temp = (int32_t)param_ptr[i];
                    memcpy(&val_to_send, &temp, 4); // 정수 비트를 float에 복사i
                } else {
                    val_to_send = param_ptr[i];
                } 
                mavlink_message_t msg;
                mavlink_msg_param_value_pack(SYSTEM_ID, COMPONENT_ID, &msg, 
                                             par.name.data(), val_to_send, par.type, PARAM::count, i);
                WIFI::dispatch_mavlink_msg(&msg);
                // pdMS_TO_TICKS(1) : 1또는 2개가 버퍼 를 넘긴다 
                // pdMS_TO_TICKS(2) : 에러가 나지 않는다.
                vTaskDelay(pdMS_TO_TICKS(2));
                i++;
            }
            break;
        }
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {   
            mavlink_param_request_read_t req;
            mavlink_msg_param_request_read_decode(msg, &req);
            if (req.target_system != SYSTEM_ID || 
                (req.target_component != COMPONENT_ID && req.target_component != 0)) {
                break;
            }
            if (req.param_index != -1) 
            {
                // 전송부 코드 예시
                float val_to_send;
                float *param_ptr =  reinterpret_cast<float*>(&PARAM::values);
                if (PARAM::params[req.param_index].type == 6) { // INT32
                    
                    int32_t temp = (int32_t)param_ptr[req.param_index]; 
                    memcpy(&val_to_send, &temp, 4); // 정수 비트를 float에 복사
                } else {
                    val_to_send = param_ptr[req.param_index];
                }
                mavlink_message_t msg;
                mavlink_msg_param_value_pack(SYSTEM_ID, COMPONENT_ID, &msg, 
                                            PARAM::params[req.param_index].name.data(),
                                            val_to_send, 
                                            PARAM::params[req.param_index].type,
                                            PARAM::count,
                                            req.param_index);               
                WIFI::dispatch_mavlink_msg(&msg);
            } 
            break;
        }
        // 2. QGC에서 특정 값을 수정하고 "자, 이거로 바꿔!"라고 할 때
        case MAVLINK_MSG_ID_PARAM_SET: {
            mavlink_param_set_t set;
            mavlink_msg_param_set_decode(msg, &set);
            if (set.target_system != SYSTEM_ID || 
                (set.target_component != COMPONENT_ID && set.target_component != 0)) {
                break;
            }
            // [중요] 변경된 값을 다시 보내줘야 QGC 화면에서 수치가 확정됨
            float val_to_send;
            if(size_t index = PARAM::find_param(set.param_id);index != -1){
                if (set.param_type == 6){    
                    int32_t temp = (int32_t)set.param_value; 
                    memcpy(&val_to_send, &temp, 4); // 정수 비트를 float에 복사
                }
                else{
                    val_to_send = (int32_t)set.param_value;
                }
                PARAM::update_param(index,val_to_send);
                // 적용 가능한 PID 계수가 있다면 즉시 동기화
                PID::sync_pid_from_params();

                mavlink_message_t msg;
                mavlink_msg_param_value_pack(SYSTEM_ID, COMPONENT_ID, &msg, 
                                            set.param_id, val_to_send, set.param_type, PARAM::count,index);               
                WIFI::dispatch_mavlink_msg(&msg);
            }
            break;
        }

        // 3. 시동(ARM)이나 특정 명령을 내릴 때
        case MAVLINK_MSG_ID_COMMAND_LONG: { //76
            mavlink_command_long_t cmd;
            mavlink_msg_command_long_decode(msg, &cmd);       
            
            // 나에게 온것이 아니면 처리하지 않음.
            if (cmd.target_system != SYSTEM_ID || 
                (cmd.target_component != COMPONENT_ID && cmd.target_component != 0)) {
                break;
            }
            switch (cmd.command){
                case MAV_CMD_COMPONENT_ARM_DISARM:{ //400
                    MAV_CMD_COMPONENT_ARM_DISARM_func(msg,cmd);
                    break;
                }
                case MAV_CMD_NAV_TAKEOFF:{ //22
                    MAV_CMD_NAV_TAKEOFF_func(msg,cmd);
                    break;
                }
                case MAV_CMD_DO_SET_HOME:{ //179
                    MAV_CMD_DO_SET_HOME_func(msg,cmd);
                    break;
                }
                // case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:{ //520
                //     MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_func(msg,cmd);
                //     break;
                // }
                case MAV_CMD_REQUEST_MESSAGE:{  //512
                    MAV_CMD_REQUEST_MESSAGE_func(msg,cmd);    
                    break;
                }
                case MAV_CMD_PREFLIGHT_CALIBRATION:{ //241  RC RADIO CALIBRATION.
                    MAV_CMD_PREFLIGHT_CALIBRATION_func(msg,cmd);
                    break;
                }
                case MAV_CMD_SET_MESSAGE_INTERVAL:{ //511
                    MAV_CMD_SET_MESSAGE_INTERVAL_func(msg,cmd);  
                    break;
                }
                case MAV_CMD_REQUEST_PROTOCOL_VERSION:{ //519
                    MAV_CMD_REQUEST_PROTOCOL_VERSION_func(msg,cmd);
                    break;
                } 
                case MAV_CMD_REQUEST_CAMERA_INFORMATION:{ //521
                    send_mav_ack(cmd.command, MAV_RESULT_UNSUPPORTED,100,0,msg->sysid,msg->compid);             
                    break;
                }
                case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:{
                    send_mav_ack(cmd.command, MAV_RESULT_ACCEPTED,100,0,msg->sysid,msg->compid);             
                    if (std::abs(cmd.param1 - 1.0f) < 0.01f) {
                        // 1번 소리(시동 성공음 등)를 짧게 울리고 재부팅하면 상태 확인에 좋습니다.
                        BUZZ::sound_system_off(); 
                        esp_restart();
                    }
                    break;
                }
                default:{
                    ESP_LOGI("TELETETRY", "QGC Message : %s , Command : %d  param1 : %f  param2 : %f ",ret_msg->name , cmd.command, cmd.param1, cmd.param2);
                    send_mav_ack(cmd.command, MAV_RESULT_UNSUPPORTED,100,0,msg->sysid,msg->compid);             

                }
                    
            } // end of switch
            
            break;
        }
        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: {
            mavlink_message_t ack_msg;
            mavlink_msg_mission_ack_pack(
                SYSTEM_ID, COMPONENT_ID,
                &ack_msg,
                msg->sysid, msg->compid,        // 받는 사람 (GCS)
                MAV_MISSION_ACCEPTED,           // 결과: 잘 지웠어!
                MAV_MISSION_TYPE_MISSION        // 어떤 타입의 미션인지
            );
            WIFI::dispatch_mavlink_msg(&ack_msg);
            break;
        }
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
            mavlink_message_t ack_msg;
            mavlink_msg_mission_count_pack(
                SYSTEM_ID, COMPONENT_ID,
                &ack_msg,
                msg->sysid, msg->compid,    // 받는 사람 (GCS)
                0,                          // 미션 총 개수
                mavlink_msg_mission_request_list_get_mission_type(msg)        //MAV_MISSION_TYPE_MISSION  //미션 타입 지정
            );
            WIFI::dispatch_mavlink_msg(&ack_msg);
            break;
        }
        case MAVLINK_MSG_ID_SET_MODE:{
            // 각 모드별로 outer loop, inner loop을 처리하는 기준이 되어진다.            
            // manual, alt control, pos control, offboard, acro, rattitude, stabilize, standby, mission, return, land
            // 모드별 pid 제어가 달라진다.            
            // 기본적인것이 끝나면 처리할것.
            mavlink_set_mode_t  cmd;
            mavlink_msg_set_mode_decode(msg, &cmd);
            
            if( cmd.target_system != SYSTEM_ID) break;
            g_heartbeat.base_mode = cmd.base_mode;
            if (g_heartbeat.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
                switch (cmd.custom_mode) {
                    case (uint32_t)0x00010000: // Manual 
                        //qgc용
                        g_heartbeat.custom_mode = (uint32_t)0x00010000; 
                        //fc용
                        g_sys.flight_mode = MODE_MANUAL;                       
                        break;
                    case (uint32_t)0x00020000: // Altitude control
                        g_heartbeat.custom_mode = (uint32_t)0x00020000;
                        g_sys.flight_mode = MODE_ALTCTL;                       
                        break;
                    case (uint32_t)0x00030000: // position control
                        g_heartbeat.custom_mode = (uint32_t)0x00030000;
                        g_sys.flight_mode = MODE_POSCTL;                       
                        break;
                    case (uint32_t)0x00040000: // Offboard
                        g_heartbeat.custom_mode = (uint32_t)0x00040000;
                        g_sys.flight_mode = MODE_OFFBOARD;                       
                        break;
                    case (uint32_t)0x00050000: // Acro
                        g_heartbeat.custom_mode = (uint32_t)0x00050000;
                        g_sys.flight_mode = MODE_ACRO;                       
                        break;
                    // case (uint32_t)0x00060000: // rattitude
                    //     g_heartbeat.custom_mode = (uint32_t)0x00060000;
                    //     g_sys.flight_mode = MODE_MANUAL;                       
                    //     break;    
                    case (uint32_t)0x00070000: // Stabilize
                        g_heartbeat.custom_mode = (uint32_t)0x00070000;
                        g_sys.flight_mode = MODE_STABILIZED;                       
                        break;
                    // case (uint32_t)0x03040000: // standby
                    //     g_heartbeat.custom_mode = (uint32_t)0x03040000;
                    //     g_sys.flight_mode = MODE_MANUAL;                       
                    //     break;
                    case (uint32_t)0x04040000: // Mission
                        g_heartbeat.custom_mode = (uint32_t)0x04040000;
                        g_sys.flight_mode = MODE_MISSION;                       
                        break; 
                    case (uint32_t)0x05040000: // Return 
                        g_heartbeat.custom_mode = (uint32_t)0x05040000;
                        g_sys.flight_mode = MODE_RTL;                       
                        break;
                    case (uint32_t)0x09040000: // Land
                        g_sys.flight_mode = MODE_PRECISION_LAND;                       
                        g_heartbeat.custom_mode = (uint32_t)0x09040000;
                        break;
                    default:
                            ESP_LOGI("MODE", "Unknown custom mode: 0x%08X", cmd.custom_mode);
                        break;
                }
            }
            break;
        }
        default:{            
           ESP_LOGI("TELETETRY", "MAVLINK msgid: %u (%s)", msg->msgid, ret_msg ? ret_msg->name : "Unknown");
        }
    }
}



// telemetry_task에서 시동 상태에 따라 시스템 상태를 관리하는 로직이 이미 구현되어 있기 때문에, 
// 여기서는 시동 상태만 업데이트하고 ACK만 보내도록 수정합니다.
void MAV_CMD_COMPONENT_ARM_DISARM_func(mavlink_message_t *msg, mavlink_command_long_t cmd){
    send_mav_ack(cmd.command, MAV_RESULT_ACCEPTED,0,0,msg->sysid,msg->compid); 
    //시동이 자동으로 꺼지니 일단 막아놓는다.
    if (cmd.param1 > 0.5f && cmd.param1 < 1.5f) {
        g_sys.is_armed = true;
    } else if (cmd.param1 < 0.5f) {
        g_sys.is_armed = false;
    }
    ESP_LOGI("TEMETETRY","Param1: %8.5f",cmd.param1);
}

// 파라미터	명칭	설명
// Param 1	Pitch	이륙 시 유지할 최소 피치 각도 (단위: 도, Degree). 기체가 상승하며 앞/뒤로 기울어지는 정도를 제어합니다.
// Param 2	Empty	비어 있음 (사용되지 않음).
// Param 3	Empty	비어 있음 (사용되지 않음).
// Param 4	Yaw	이륙 시 유지할 방향 (단위: 도). 보통 현재 헤딩(방향)을 유지하려면 NaN 혹은 0을 사용합니다.
// Param 5	Latitude	이륙 지점의 위도 (Target Latitude). 0이면 현재 위치를 사용합니다.
// Param 6	Longitude	이륙 지점의 경도 (Target Longitude). 0이면 현재 위치를 사용합니다.
// Param 7	Altitude	이륙 목표 고도 (단위: 미터, m). 지면으로부터의 상대 고도(Relative Altitude)를 의미합니다.
// 각 파라메터의 값에 따라 takeoff시에 고도를 얼마에 유지하면 안정적인 상황에서 대기 상태를 유지할 수 있을지 결정하는데 사용됩니다.
void MAV_CMD_NAV_TAKEOFF_func(mavlink_message_t *msg, mavlink_command_long_t cmd){    
    send_mav_ack(cmd.command, MAV_RESULT_ACCEPTED,0,0,msg->sysid,msg->compid);    
    // float tmp_yaw = cmd.param4; // yaw 각도 (deg)
    // float tmp_lat = cmd.param5; // 위도 (deg)
    // float tmp_lon = cmd.param6; // 경도 (deg)
    // float tmp_alt = cmd.param7; // 고도 (m)
    ESP_LOGI("PROCESS_CMD_LONG","MSG : (%d), CMD : (%d), PARAM1 :(%f), PARAM4 : (%f), PARAM5 : (%f), PARAM6 : (%f), PARAM7 : (%f)",
        msg->msgid,cmd.command,cmd.param1,cmd.param4,cmd.param5,cmd.param6,cmd.param7);

}


void MAV_CMD_DO_SET_HOME_func(mavlink_message_t *msg, mavlink_command_long_t cmd){
    send_mav_ack(cmd.command, MAV_RESULT_ACCEPTED,0,0,msg->sysid,msg->compid);
    if (cmd.param1 == 1){
        // Param 1이 1이면 현재 센서(GPS) 위치를 홈으로 설정
        qgc_home_pos.lat = g_gps.lat;
        qgc_home_pos.lon = g_gps.lon;
        qgc_home_pos.alt = g_gps.alt;
    }
    else
    {
        // Param 1이 0이면 전달받은 파라미터로 설정
        qgc_home_pos.lat = cmd.param5;
        qgc_home_pos.lon = cmd.param6;
        qgc_home_pos.alt = cmd.param7;
    }
    qgc_home_pos.is_set = true;

    //처리결과 송싱 해야함...
    
}

void MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES_func(mavlink_message_t *msg, mavlink_command_long_t cmd){
    send_mav_ack(cmd.command, MAV_RESULT_ACCEPTED,0,0,msg->sysid,msg->compid);

    // 1. ACK 패킹 및 전송
    // 2. 버전 정보(AUTOPILOT_VERSION) 설정
    mavlink_autopilot_version_t version = {};
    // 핵심: 지원하는 기능을 비트로 나열
    version.capabilities = MAV_PROTOCOL_CAPABILITY_MAVLINK2 | 
                        MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
                        MAV_PROTOCOL_CAPABILITY_MISSION_INT |
                        MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET;
    version.board_version = 1;       
    version.flight_sw_version = 0x010D0000; // v1.13.0 (PX4 스타일 버전 넘버링)
    version.middleware_sw_version = 0x010D0000;
    version.os_sw_version = 0x00000000; // FreeRTOS 등 사용 시 기입 가능
    version.vendor_id = 0x1234; // 필요 시 수정
    version.product_id = 0x5678;
    // 필요 시 여기에 UID 등을 추가로 채웁니다.

    // 3. 버전 정보 패킹 및 전송
    mavlink_message_t ver_msg;
    mavlink_msg_autopilot_version_encode(SYSTEM_ID, COMPONENT_ID, &ver_msg, &version);
    WIFI::dispatch_mavlink_msg(&ver_msg);

}

//QGC에서 MESSAGE : 76 , Command : 512 
void MAV_CMD_REQUEST_MESSAGE_func(mavlink_message_t *msg, mavlink_command_long_t cmd)  
{
    send_mav_ack(cmd.command, MAV_RESULT_ACCEPTED,0,0,msg->sysid,msg->compid);
    uint32_t requested_id = static_cast<uint32_t>(cmd.param1);
    switch (requested_id)
    {
    case MAVLINK_MSG_ID_PROTOCOL_VERSION: // 519
        // 1. ACK 패킹 및 전송
        mavlink_message_t ack_msg;
        mavlink_msg_protocol_version_pack(
            SYSTEM_ID,           // 내 FC 시스템 ID (보통 1)
            COMPONENT_ID,        // 내 컴포넌트 ID (MAV_COMP_ID_AUTOPILOT1: 1)
            &ack_msg,
            200,                 // version: MAVLink 2.0 (200)
            100,                 // min_hw_version: 최소 지원 버전 (100)
            200,                 // max_hw_version: 최대 지원 버전 (200)
            0,                   // spec_version_hash: 보통 0 (또는 git hash의 일부)
            0                    // library_version_hash: 보통 0 (또는 git hash의 일부)
        );
        WIFI::dispatch_mavlink_msg(&ack_msg);
        break;
    case 280: case 259: case 148: case 435: case 397: case 395:
        send_mav_ack(cmd.command, MAV_RESULT_UNSUPPORTED,0,0,msg->sysid,msg->compid);
        break;
    default:
        send_mav_ack(cmd.command, MAV_RESULT_UNSUPPORTED,0,0,msg->sysid,msg->compid);
           ESP_LOGI("PROCESS_CMD_LONG", "QGC Message : %d , Command : %d , Request Id: %d",msg->msgid , cmd.command , requested_id);
    }
    
}

void MAV_CMD_PREFLIGHT_CALIBRATION_func(mavlink_message_t *msg, mavlink_command_long_t cmd){
    bool calibrate_gyro   = (cmd.param1 == 1.0f);
    bool calibrate_mag    = (cmd.param2 == 1.0f);
    bool calibrate_level  = (cmd.param3 == 1.0f);
    bool calibrate_accel  = (cmd.param4 == 1.0f);
    bool calibrate_airspeed = (cmd.param5 == 1.0f);
    
    send_mav_ack(cmd.command, MAV_RESULT_UNSUPPORTED,0,0,msg->sysid,msg->compid);


    // TODO: 센서 캘리브레이션 요청 처리 구현
    if (calibrate_gyro || calibrate_mag || calibrate_level || calibrate_accel || calibrate_airspeed) {
        ESP_LOGI("CALIB", "Preflight calibration requested: gyro=%d mag=%d level=%d accel=%d airspeed=%d",
                 calibrate_gyro, calibrate_mag, calibrate_level, calibrate_accel, calibrate_airspeed);
    }
}



void MAV_CMD_SET_MESSAGE_INTERVAL_func(mavlink_message_t *msg, mavlink_command_long_t cmd) {
    // Param 1: 메시지 ID (예: MAVLINK_MSG_ID_ATTITUDE 등)
    // Param 2: 전송 간격 (마이크로초, us 단위)
    // -1: 전송 중지
    // 0: 기본 간격 사용
    // 100000: 100,000us = 0.1초 (10Hz)
    uint32_t msg_id = static_cast<uint32_t>(cmd.param1);
    float interval_us = cmd.param2;

    static bool imu_data_sending = false;
    static bool att_data_sending = false;
    static bool local_pos_data_sending = false;
    static bool pos_target_data_sending = false;
    
    send_mav_ack(cmd.command, MAV_RESULT_UNSUPPORTED,0,0,msg->sysid,msg->compid);


    switch (msg_id) {
        case 83: // MAVLINK_MSG_ID_RAW_IMU
            imu_data_sending = (interval_us > 0.0f);
            ESP_LOGI("PROCESS_CMD_LONG", "MAVLINK_MSG_ID_RAW_IMU interval set to %f us, sending: %s", interval_us, imu_data_sending ? "ON" : "OFF");
            break;
        case 31: // MAVLINK_MSG_ID_ATTITUDE_QUATERNION
            att_data_sending = (interval_us > 0.0f);
            ESP_LOGI("PROCESS_CMD_LONG", "MAVLINK_MSG_ID_ATTITUDE_QUATERNION interval set to %f us, sending: %s", interval_us, att_data_sending ? "ON" : "OFF");
            break;
        case 32: // MAVLINK_MSG_ID_LOCAL_POSITION_NED
            local_pos_data_sending = (interval_us > 0.0f);
            ESP_LOGI("PROCESS_CMD_LONG", "MAVLINK_MSG_ID_LOCAL_POSITION_NED interval set to %f us, sending: %s", interval_us, local_pos_data_sending ? "ON" : "OFF");
            break;
        case 85: // MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED
            pos_target_data_sending = (interval_us > 0.0f);
            ESP_LOGI("PROCESS_CMD_LONG", "MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED interval set to %f us, sending: %s", interval_us, pos_target_data_sending ? "ON" : "OFF");
            break;
        default:
            break;
    }
}


void MAV_CMD_REQUEST_PROTOCOL_VERSION_func(mavlink_message_t *msg, mavlink_command_long_t cmd){
    send_mav_ack(cmd.command, MAV_RESULT_ACCEPTED,0,0,msg->sysid,msg->compid);
    mavlink_message_t ack_msg;
    mavlink_msg_protocol_version_pack(
        SYSTEM_ID,           // 내 FC 시스템 ID (보통 1)
        COMPONENT_ID,        // 내 컴포넌트 ID (MAV_COMP_ID_AUTOPILOT1: 1)
        &ack_msg,
        200,                 // version: MAVLink 2.0 (200)
        100,                 // min_hw_version: 최소 지원 버전 (100)
        200,                 // max_hw_version: 최대 지원 버전 (200)
        0,                   // spec_version_hash: 보통 0 (또는 라이브러리 생성 해시)
        0                    // library_version_hash: 보통 0
    );
    WIFI::dispatch_mavlink_msg(&ack_msg);
}
}