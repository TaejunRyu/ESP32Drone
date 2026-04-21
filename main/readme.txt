qgc에서 디버깅하는 방법ㄴ

1. STATUSTEXT 메시지 사용 (가장 간단함)
    QGC 상단의 알림 창(Console)이나 음성 안내로 텍스트 메시지를 띄우는 방법입니다. 코드의 특정 지점을 통과했는지 확인할 때 좋습니다.    
    특징: 문자열(최대 50자)을 보낼 수 있습니다.
    표시: QGC 화면 상단 알림 바 또는 Vehicle View -> Messages 탭.


        mavlink_message_t msg;
        char text[] = "Arming Sequence Started!";
        mavlink_msg_statustext_pack(system_id, component_id, &msg, MAV_SEVERITY_DEBUG, text, 0, 0);
        // 이후 UDP 브로드캐스트(192.168.4.255)로 전송

        MAV_SEVERITY_INFO: "IMU Fusion: Dual Mode Active"
        MAV_SEVERITY_WARNING: "IMU1 Jitter Detected - High Latency"
        MAV_SEVERITY_CRITICAL: "IMU1 Failure! Switching to IMU2" (복구 실행 시)






2. NAMED_VALUE_FLOAT 사용 (실시간 그래프 확인용) 🚀 강력 추천
    변수 이름과 함께 실시간 숫자 데이터를 보낼 때 사용합니다. QGC의 Analyze Tools에서 실시간 그래프로 그릴 수 있어 PID 튜닝이나 센서 값 모니터링에 최고입니다.
    특징: 이름(10자 이내) + float 값을 묶어서 전송.
    확인 방법: QGC 메뉴 -> Analyze Tools -> Mavlink Inspector 또는 Realtime Plot에서 해당 이름의 변수를 체크.

        mavlink_message_t msg;
        float my_debug_val = 12.34f; 
        mavlink_msg_named_value_float_pack(system_id, component_id, &msg, 
                                        millis(), "Battery_V", my_debug_val);
        // 전송 루틴...

3. DEBUG_VECT 사용 (3축 데이터용)
    가속도, 자이로, 혹은 XYZ 좌표 같은 3개 세트 데이터를 한 번에 보낼 때 유리합니다.
    특징: 이름 + x, y, z 값 전송.
        mavlink_msg_debug_vect_pack(system_id, component_id, &msg, 
                            "PID_ERR", millis(), err_x, err_y, err_z);





// task간의 데이터 고오유 방식  넘길데이터가 많을경우 타이밍이 달라서 변수들을 수정중 가져가게 되면 이 값들의 생성 타이밍이 달라질수 있다
// 그러므로 업데이트 동안에는 건들지말게끔 뮤텍스를 걸어서 최악의 상황을 방지한다.
// 공유 데이터 구조체
typedef struct {
    float yaw;     // 진북 보정된 값
    float heading; // 0~360도 값
} shared_data_t;

shared_data_t g_nav_data;
SemaphoreHandle_t xNavMutex;

// [Task 1: Sensor & Fusion (Priority: High)]
void sensor_task(void *pv) {
    while(1) {
        // ... (순천 자편각 7.7도 더하기 등 모든 연산 완료) ...
        if (xSemaphoreTake(xNavMutex, 0) == pdTRUE) { // 기다리지 않고 즉시 시도
            g_nav_data.yaw = yaw_rad * RAD_TO_DEG;
            g_nav_data.heading = heading_deg;
            xSemaphoreGive(xNavMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // 200Hz 주기로 반복
    }
}

// [Task 2: Telemetry/QGC (Priority: Low)]
void telemetry_task(void *pv) {
    shared_data_t local_copy;
    while(1) {
        if (xSemaphoreTake(xNavMutex, portMAX_DELAY) == pdTRUE) {
            local_copy = g_nav_data; // 스냅샷 복사
            xSemaphoreGive(xNavMutex);
        }
        // 이제 local_copy를 사용해 느긋하게 QGC 전송 작업 수행
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz 주기로 반복
    }
}






        // y,x,z 축의 재정렬
        //calulation_mag_x = raw_mag_y;
        //calulation_mag_y = raw_mag_x;
        //calulation_mag_z = raw_mag_z;

        // y,z,x 축의 재정렬
        //calulation_mag_x = raw_mag_y;
        //calulation_mag_y = raw_mag_z;
        //calulation_mag_z = raw_mag_x;

        // z,x,y 축의 재정렬
        //calulation_mag_x = raw_mag_z;
        //calulation_mag_y = raw_mag_x;
        //calulation_mag_z = raw_mag_y;

        // z,y,x 축의 재정렬
        //calulation_mag_x = raw_mag_z;
        //calulation_mag_y = raw_mag_y;
        //calulation_mag_z = raw_mag_x;

        //45도 
        //calulation_mag_x = ( raw_mag_x - raw_mag_y) * 1/sqrt(2);
        //calulation_mag_y = ( raw_mag_x + raw_mag_y) * 1/sqrt(2);

        // 90도 회전  
        //calulation_mag_x = raw_mag_y *   1.0f;
        //calulation_mag_y = raw_mag_x *  -1.0f;
        
        // 135도 회전  
        //calulation_mag_x = - ( raw_mag_x + raw_mag_y) * 1/sqrt(2);
        //calulation_mag_y =   ( raw_mag_x - raw_mag_y) * 1/sqrt(2);

        // 180도 회전  
        //calulation_mag_x = calulation_mag_x *  -1.0f;
        //calulation_mag_y = calulation_mag_y *  -1.0f;

        // 225도 회전  
        //calulation_mag_x =   ( -raw_mag_x + raw_mag_y) * 1/sqrt(2);
        //calulation_mag_y =  -(  raw_mag_x + raw_mag_y) * 1/sqrt(2);

        // 270도 회전  
        //calulation_mag_x = raw_mag_y *  -1.0f;
        //calulation_mag_y = raw_mag_x *   1.0f;

        // ROLL 180도
        //calulation_mag_z = raw_mag_z *  -1.0f;


        // temp_val = calulation_mag_x;
        // calulation_mag_x =  calulation_mag_y ;
        // calulation_mag_y = temp_val ;


        // calulation_mag_x = -calulation_mag_x;
        // calulation_mag_y = -calulation_mag_y;
        // calulation_mag_z = -calulation_mag_z ;
