#include "ryu_timer.h"


namespace TIMER {


static void vDroneTimerCallback(TimerHandle_t xTimer) {
    static uint8_t step = 0;
    mavlink_message_t msg;

    // 10hz로 구분하고 있으므로 매번 처리...
    mavlink_msg_attitude_pack(SYSTEM_ID, COMPONENT_ID, &msg, esp_timer_get_time()/1000, 
                                                -g_attitude.roll   * DEG_TO_RAD, 
                                                -g_attitude.pitch  * DEG_TO_RAD, 
                                                g_attitude.yaw    * DEG_TO_RAD, 
                                                g_attitude.rollspeed    * DEG_TO_RAD, 
                                                g_attitude.pitchspeed   * DEG_TO_RAD, 
                                                g_attitude.yawspeed     * DEG_TO_RAD );
    WIFI::dispatch_mavlink_msg(&msg);

    static gps_data_t m_gps={};
    static uint32_t last_itow = 0;     // 마지막으로 전송한 iTOW 저장



 {
        // 1. RC 채널 데이터 가상 생성 (1000~2000 사이의 값)
        uint16_t channels[18] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 
                                UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, 
                                UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, 
                                UINT16_MAX, UINT16_MAX};
        // 2. 메시지 패키징 (ID: 65)
        // time_boot_ms: 부팅 후 시간, chancount: 채널수, rssi: 신호세기(0~255)
        // mavlink_msg_rc_channels_raw_pack(
        //     SYSTEM_ID,      // System ID (드론 ID, 보통 1)
        //     COMPONENT_ID,      // Component ID (보통 1: Autopilot)
        //     &msg, 
        //     esp_log_timestamp(), // time_boot_ms
        //     0,                   // chancount (8채널 사용)
        //     channels[0], channels[1], channels[2], channels[3], 
        //     channels[4], channels[5], channels[6], channels[7],                    
        //     100                  // RSSI (신호 세기 0~255)
        // );
        mavlink_msg_rc_channels_pack(
            SYSTEM_ID,      // System ID (드론 ID, 보통 1)
            COMPONENT_ID,      // Component ID (보통 1: Autopilot)
            &msg, 
            esp_log_timestamp(), // time_boot_ms
            8,                   // chancount (8채널 사용)
            channels[0], channels[1], channels[2], channels[3], 
            channels[4], channels[5], channels[6], channels[7],
            channels[8], channels[9], channels[10], channels[11],
            channels[12], channels[13], channels[14], channels[15],
            channels[16], channels[17],
            200                  // RSSI (신호 세기 0~255)
        );
        WIFI::dispatch_mavlink_msg(&msg);
        //break;                        
}



























    switch(step){

        case 1: case 8: case 9:{
            // g_gps 공동 변수에서 읽어와 사용한다.
            // 정확한 데이터 보장이 필요하기 때문에 시간별 다름을 없애는 목적......
            // gps시간이 다르면 복사하고 아니면 이전 데이터 사용
            if (xSemaphoreTake(GPS::xGpsMutex, 0 )== pdTRUE) {
                if (g_gps.iTOW != last_itow){
                    m_gps       = g_gps;
                    last_itow   = g_gps.iTOW;         
                    m_gps.last_update_tick = xTaskGetTickCount();      
                }
                xSemaphoreGive(GPS::xGpsMutex);
            }
        }
    }

    switch (step) {
        case 0:{ // 하트비트 전송
            mavlink_msg_heartbeat_pack(SYSTEM_ID, COMPONENT_ID,&msg, 
                                        MAV_TYPE_QUADROTOR, 
                                        //MAV_AUTOPILOT_GENERIC,
                                        MAV_AUTOPILOT_PX4,
                                        g_heartbeat.base_mode,  
                                        g_heartbeat.custom_mode, 
                                        g_sys.system_status);
            WIFI::dispatch_mavlink_msg(&msg);
            break;
        }
        case 3:{ // 시스템 상태 전송 (배터리, 전압 등)
            uint32_t sensors_present = 
                        MAV_SYS_STATUS_SENSOR_3D_ACCEL | 
                        MAV_SYS_STATUS_SENSOR_3D_ACCEL2 | // 두 번째 IMU 가속도
                        MAV_SYS_STATUS_SENSOR_3D_GYRO |
                        MAV_SYS_STATUS_SENSOR_3D_GYRO2 |
                        MAV_SYS_STATUS_SENSOR_3D_MAG |
                        MAV_SYS_STATUS_SENSOR_3D_MAG2| 
                        MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE  |
                        MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE|
                        MAV_SYS_STATUS_AHRS|
                        MAV_SYS_STATUS_SENSOR_BATTERY|
                        MAV_SYS_STATUS_SENSOR_RC_RECEIVER
                        ;   // 두 번째 IMU 자이로
            uint32_t sensors_enabled = sensors_present; // 모두 활성화됨
            uint32_t sensors_health  = sensors_present;  // 모두 정상(Healthy)

            // 1. CPU Load 계산 (0 ~ 1000 사이의 값으로 변환)
            // 로그상 1700us / 2500us 라면 약 680이 됨
            uint16_t load = (uint16_t)((FLIGHT::total_us * 1000) / FLIGHT::INTERVAL_US);
            
            // 2. 배터리 가짜 데이터 (12.6V, 10.5A, 85% 잔량)
            uint16_t battery_voltage = (uint16_t)(BATT::get_battery_voltage() * 1000.0f); //mv
            int16_t current_battery = 1050;   // [10mA 단위, 즉 10.5A]
            int8_t battery_remaining = 85;    // [%]
                uint16_t comms_drop_rate = 0;                          // 통신 패킷 드랍률 (0.01% 단위)
                uint16_t comms_errors = 0;                             // 통신 에러 횟수
            // 시스템 상태 패킷 구성 예시
            mavlink_msg_sys_status_pack(
                SYSTEM_ID, COMPONENT_ID, &msg, 
                sensors_present, sensors_enabled, sensors_health,         // 센서 상태 비트마스크
                load,        // CPU Load (0~1000)
                battery_voltage, current_battery, battery_remaining, 
                comms_drop_rate, comms_errors, 0, 0, 0, 0,0,0,0
            );
            WIFI::dispatch_mavlink_msg(&msg);
            break;
        }    
        case 6:{ // 라디오 상태 전송 (RSSI, Noise)
             mavlink_msg_radio_status_pack_chan(
                            SYSTEM_ID, COMPONENT_ID,MAVLINK_COMM_1, &msg, 
                            WIFI::current_rssi, // 드론이 받은 브릿지의  신호
                            0,0, WIFI::noise_floor, 0, 0, 0);
            WIFI::dispatch_mavlink_msg(&msg);
            break;
        }
        case 9:{ // gps 정보 
            if (m_gps.home_alt > -9000.0f && m_gps.fixType >= 3) {
                mavlink_msg_gps_raw_int_pack(
                        SYSTEM_ID, COMPONENT_ID, &msg, 
                        esp_timer_get_time() / 1000,               
                        m_gps.fixType,                              // 실제 Fix 타입을 그대로 전달 (0~4)                     
                        static_cast<int32_t>(m_gps.lat * 1e7),      // 위도
                        static_cast<int32_t>(m_gps.lon * 1e7),      // 경도
                        static_cast<int32_t>(m_gps.hMSL), // 해발 고도 (MSL, mm)
                        static_cast<uint16_t>(m_gps.pDOP),          
                        static_cast<uint16_t>(m_gps.pDOP),          // VDOP 대신 pDOP 사용 가능
                        static_cast<uint16_t>(m_gps.gSpeed),        // 지표속도
                        static_cast<uint16_t>(m_gps.headMot),       // 이동방향
                        static_cast<uint8_t>(m_gps.sats),           // 위성수
                        static_cast<int32_t>(m_gps.height),         // alt_ellipsoid (mm 단위 그대로)
                        m_gps.hAcc,                                 // 수평 정확도 (mm)
                        m_gps.vAcc,                                 // 수직 정확도 (mm)
                        m_gps.sAcc,                                 // 속도 정확도 (mm/s)
                        0,                                          // hdg_acc – [degE5] Heading / track uncertainty
                        static_cast<uint16_t>(g_attitude.heading * 100.0f) // yaw (cdeg 단위로 변환)
                    );
                    WIFI::dispatch_mavlink_msg(&msg);
            }
            break;
        }
        case 1: case 8:{
            if (m_gps.home_alt > -9000.0f && m_gps.fixType >= 3) {
                //현재고도
                int32_t alt_msl = static_cast<int32_t>(m_gps.hMSL );
                
                // 상대 고도 (Relative) mm 단위
                int32_t alt_rel = static_cast<int32_t>((m_gps.hMSL - m_gps.home_alt) );
                
                mavlink_msg_global_position_int_pack(
                    SYSTEM_ID, COMPONENT_ID, &msg, esp_timer_get_time()/1000,
                    static_cast<int32_t>(m_gps.lat * 1e7), 
                    static_cast<int32_t>(m_gps.lon * 1e7),
                    static_cast<int32_t>(alt_msl),      // 해수면 고도
                    static_cast<int32_t>(alt_rel),      // 이것은 기압계로 측정한 고도 => g_baro.filtered_altitude * 1000.0f),
                    static_cast<int16_t>(m_gps.velN),   // 단위(cm/s) gps에서 데이터를 받아 처리 VGT문장에서 받으면 된다.
                    static_cast<int16_t>(m_gps.velE),
                    static_cast<int16_t>(m_gps.velD),
                    static_cast<int16_t>(g_attitude.heading * 100.0f));
                    WIFI::dispatch_mavlink_msg(&msg);
            }
            break;
        }
        case 2:
        case 4:
        case 5:
        case 7:
            break;
    }
    // 0 -> 1 -> 2 순환
    step = (step + 1) % DIVIDE_COUNT;
}


void setupTimer() {
    // 1000ms / DIVIDE_COUNT = 약 xxx ms 주기로 타이머 생성
    TimerHandle_t xTimer_xMs;
    xTimer_xMs = xTimerCreate(
            "xTimer_xMs", 
            pdMS_TO_TICKS(DIVIDE_TIME), 
            pdTRUE, (void*)0, 
            vDroneTimerCallback);
    if (xTimer_xMs != NULL) {
        xTimerStart(xTimer_xMs, 0);
    }
}
} //namespace TIMER