#include "ryu_telemetry.h"

#include <esp_log.h>
#include <c_library_v2/common/mavlink.h>
#include "ryu_mavlink.h"
#include "ryu_wifi.h"

namespace Service
{

// static const char *TAG = "TELEMETRY";
   
// static mavlink_status_t status;

// void telemetry_task(void *pv) {

//     auto& mavlink =  Service::Mavlink::get_instance();

//     mavlink_message_t msg;
//     Service::EspNow::esp_now_data_t pkt;
    
//     while (true) 
//     {
//         if (xQueueReceive(Service::EspNow::get_instance().mavlink_rx_queue, &pkt, portMAX_DELAY)) {                
//             for (int i = 0; i < pkt.len; ++i) {
//                 if (mavlink_parse_char(MAVLINK_COMM_2, pkt.buffer[i], &msg, &status)) {
//                     //printf("msgid : %4d msgseq: %4d sysid: %4d compid : %4d\n",msg.msgid,msg.seq,msg.sysid,msg.compid);
//                     mavlink.handle_mavlink_message(&msg);
//                 }
//             }

//             // QGC 명령에 따른 상태 업데이트 로직
//             static bool previous_armed_state = false;
//             if (previous_armed_state != g_sys.is_armed) {
//                 if (g_sys.is_armed) {
//                     ESP_LOGD(TAG,"시동으로 프래그 변환(시동)");
//                     g_heartbeat.base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
//                     g_sys.system_status = MAV_STATE_ACTIVE;
//                     // calibrate_ground_pressure(); // 주석 처리 유지: 통신 두절 방지
//                 } else {
//                     ESP_LOGD(TAG,"시동으로 프래그 변환(시동 꺼짐)");
//                     g_heartbeat.base_mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
//                     g_sys.system_status = MAV_STATE_STANDBY;
//                 }
//                 previous_armed_state = g_sys.is_armed; // 중복 코드 제거
//             }
//         } // if(xQueueReceive(....))

//     } //while(true)

// } // telemetry_task


const char* Telemetry::TAG = "Telemetry";

Telemetry::Telemetry(){
    ESP_LOGI(TAG,"Initializing Telemetry Service...");
}

Telemetry::~Telemetry(){
}

void Telemetry::initialize()
{
    if(_initialized) return;
    //
    _initialized = true;
}

void Telemetry::telemetry_task(void *pv)
{
    auto& mavlink =  Service::Mavlink::get_instance();
    static mavlink_status_t status;
    mavlink_message_t msg;
    Service::EspNow::esp_now_data_t pkt;
    
    while (true) {
        if (xQueueReceive(EspNow::get_instance().mavlink_rx_queue, &pkt, portMAX_DELAY)) {                
            for (int i = 0; i < pkt.len; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_2, pkt.buffer[i], &msg, &status)) {
                    mavlink.handle_mavlink_message(&msg);

                    // QGC 명령에 따른 상태 업데이트 로직
                    static bool previous_armed_state = false;
                    if (previous_armed_state != g_sys.is_armed) {
                        if (g_sys.is_armed) {
                            ESP_LOGD(TAG,"시동으로 프래그 변환(시동)");
                            g_heartbeat.base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
                            g_sys.system_status = MAV_STATE_ACTIVE;
                            // calibrate_ground_pressure(); // 주석 처리 유지: 통신 두절 방지
                        } else {
                            ESP_LOGD(TAG,"시동으로 프래그 변환(시동 꺼짐)");
                            g_heartbeat.base_mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
                            g_sys.system_status = MAV_STATE_STANDBY;
                        }
                        previous_armed_state = g_sys.is_armed; // 중복 코드 제거
                    }
                }
            }
        } // if(xQueueReceive(....))
    } //while(true)
}

void Telemetry::start_task()
{
    auto res = xTaskCreatePinnedToCore(telemetry_task, "telemetry", 8192, this, 15, NULL, 0);
    if (res != pdPASS) ESP_LOGE(TAG, "❌ 3.Telemetry Task is failed! code: %d", res);
    else ESP_LOGI(TAG, "✓ 3.Telemetry task is passed...");
}

} // namespace TELEM







// pid calibration이 필요한 명령이 왔는지 확인하여 플래그 설정
// if (counter %2 == 0) {
//     if(imu_data_sending){
//         mavlink_message_t ret_msg;
//         mavlink_msg_raw_imu_pack(SYSTEM_ID, COMPONENT_ID, &ret_msg, 
//         boot_time_ms, g_imu.acc[0], g_imu.acc[1], g_imu.acc[2], g_imu.gyro[0], g_imu.gyro[1], g_imu.gyro[2], g_imu.mag[0], g_imu.mag[1], g_imu.mag[2], 0, 0);
//         dispatch_mavlink_msg(&ret_msg);
//     }
//     extern float q0, q1, q2, q3;            
//     if(att_data_sending){
//         mavlink_message_t ret_msg;
//         mavlink_msg_attitude_quaternion_pack(SYSTEM_ID, COMPONENT_ID, &ret_msg, boot_time_ms, 
//                                                 q0, q1, q2, q3, 
//                                                 g_attitude.roll * 0.01745f, g_attitude.pitch * 0.01745f, -g_attitude.yaw * 0.01745f,nullptr); 
//                                                 // roll, pitch는 양수 방향이 시계 반대방향이 되도록 변환

//         dispatch_mavlink_msg(&ret_msg);
//     }
//     if(local_pos_data_sending){
//         mavlink_message_t ret_msg;
//         mavlink_msg_local_position_ned_pack(SYSTEM_ID, COMPONENT_ID, &ret_msg, boot_time_ms,
//                                             0, 0, 0, // x, y, z 위치 (현재는 0으로 고정)
//                                             0, 0, 0); // x, y, z 속도 (현재는 0으로 고정)
//         dispatch_mavlink_msg(&ret_msg);
    
//     }
//     if(pos_target_data_sending){
//         mavlink_message_t ret_msg;
//         mavlink_msg_position_target_local_ned_pack(SYSTEM_ID, COMPONENT_ID, &ret_msg, boot_time_ms,
//                                                 0, 0, MAV_FRAME_LOCAL_NED, 
//                                                 0b0000111111000111, // 위치와 고도 제어 활성화
//                                                 0, 0, 0, // x, y, z 위치 (현재는 0으로 고정)
//                                                 0, 0, 0, // x, y, z 속도 (현재는 0으로 고정)
//                                                 0, 0, 0);   // 가속도와 yaw 제어는 사용하지 않음
//         dispatch_mavlink_msg(&ret_msg);
//     }
// }
