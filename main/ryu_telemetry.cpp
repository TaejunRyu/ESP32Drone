#include "ryu_telemetry.h"

#include <esp_log.h>
#include <c_library_v2/common/mavlink.h>
#include "ryu_mavlink.h"



namespace TELEM
{

QueueHandle_t mavlink_rx_queue = NULL;
static mavlink_status_t status;

void telemetry_task(void *pv) {

    mavlink_message_t msg;
    // uint8_t counter = 0; // 주기 제어용 카운터
    // auto xLastWakeTime = xTaskGetTickCount();
    // const auto xFrequency = pdMS_TO_TICKS(50); // 1 loop에 50ms  x 20번 = 1000ms = 1 second
    TELEM::esp_now_data_t pkt;
    
    while (true) 
    {
        if (xQueueReceive(mavlink_rx_queue, &pkt, portMAX_DELAY)) {                
            for (int i = 0; i < pkt.len; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_2, pkt.data[i], &msg, &status)) {
                    //printf("msgid : %4d msgseq: %4d sysid: %4d compid : %4d\n",msg.msgid,msg.seq,msg.sysid,msg.compid);
                    MAV::handle_mavlink_message(&msg);
                }else{                    
                    // 하나의 패킷이 parser를 실행할때 1:1d에 해당하지 않으면 ?
                }
            }

            /// QGC 명령에 따른 상태 업데이트 로직
            static bool previous_armed_state = false;
            if (previous_armed_state != g_sys.is_armed) {
                if (g_sys.is_armed) {
                    //ESP_LOGI("TELEMETRY","시동으로 프래그 변환(시동)");
                    g_heartbeat.base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
                    g_sys.system_status = MAV_STATE_ACTIVE;
                    // calibrate_ground_pressure(); // 주석 처리 유지: 통신 두절 방지
                } else {
                    //ESP_LOGI("TELEMETRY","시동으로 프래그 변환(시동 꺼짐)");
                    g_heartbeat.base_mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
                    g_sys.system_status = MAV_STATE_STANDBY;
                }
                previous_armed_state = g_sys.is_armed; // 중복 코드 제거
            }
        }

    }
}

} //namespace TELEM







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
