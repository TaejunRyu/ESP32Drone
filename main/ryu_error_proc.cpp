#include "ryu_error_proc.h"

namespace ERR{

// 태스크 핸들 (Core 0의 에러 태스크를 지칭)
TaskHandle_t xErrorHandle = NULL;
// 시스템 통합 상태 (비트마스크)
volatile uint32_t g_system_health = 0xFFFFFFFF; // 초기값은 모두 OK

void error_manager_task(void *pvParameters) {
    uint32_t notifiedValue;
    esp_err_t ret=ESP_FAIL;

    while (true) {
        // 알림이 올 때까지 무한 대기 (CPU 점유율 0%)
        if (xTaskNotifyWait(0x00, ULONG_MAX, &notifiedValue, portMAX_DELAY) == pdPASS) {

            //  IMU/MAG/BARO  모두 에러 발생 또는 I2C 에러시 모든 것 리셋.
            if (notifiedValue & ERR_I2C_BUS_HANG) {
                g_system_health &= ~SYS_HEALTH_IMU_OK;  // 상태 차단                                
                i2c_handle = I2C::i2c_bus_hardware_clear(i2c_handle); // 하드웨어 리셋 (SCL 토글)        
                if (i2c_handle !=NULL){ 
                    ret = reinit_all_sensors(i2c_handle);             // 센서 레지스터 재설정
                    auto [ret_code,macc,mgyro] =ICM20948::read_raw_data(imu_handle[0]);
                    if (ret_code == ESP_OK && (macc[0] + macc[1] + macc[2]) > 1.0f){
                        ret = ESP_OK;
                        FLIGHT::imu_error_cnt =0;
                        FLIGHT::mag_error_cnt =0;
                        FLIGHT::baro_error_cnt =0;
                        FLIGHT::imu_active_index = 0;
                        FLIGHT::mag_active_index = 0;
                        FLIGHT::baro_active_index = 0;
                        g_sys.error_hold_mode = false;                
                        // 모든 센서가 정상으로 돌아왔다고 가정하고 상태 비트 모두 켜기 (필요에 따라 개별적으로 설정할 수도 있음)
                        g_system_health |= SYS_HEALTH_IMU_OK|SYS_HEALTH_MAG_OK|SYS_HEALTH_BARO_OK;                    
                    }
                }else{
                    ret = ESP_FAIL;
                }

                if (ret != ESP_OK){
                    SERVO::stop_all_motors();
                    while(true){
                        BUZZ::sound_emergency(); // 나 죽었다~~~~~~~~
                    }
                }
                printf("⚠️ WARNING: IMU/MAG/BARO Not Working Detected!\n");
            }

            // 2. 조종기 신호 상실 처리
            if (notifiedValue & ERR_RC_LOST) {
                BUZZ::sound_emergency();
                g_sys.error_hold_mode = false;                
                // Telemetry로 경고 전송 및 로그 저장 로직
                //save_error_to_nvs("RC_LOST");
                printf("⚠️ WARNING: Lost Control Signal Detected!\n");
            }

            // 3. 배터리 저전압 처리
            if (notifiedValue & ERR_BATTERY_LOW) {
                BUZZ::sound_low_battery();
                printf("⚠️ WARNING: Low Battery Detected!\n");
            }
            
            // 4. GPS TIMEOUT
            if (notifiedValue & ERR_GPS_TIMEOUT) {
                //BUZZ::sound_emergency();
                g_sys.error_hold_mode = false;                
                printf(" {(%lld): ⚠️ WARNING: Gps Timeout Detected!\n",esp_timer_get_time());
            }

            // 처리 완료 후 로그 출력
            //std::printf("Error Resolved: 0x%08x\n", (unsigned int)notifiedValue);
        }
    }
}

// 각 센서의 초기화 함수들을 모아놓은 메인 함수 (test 완료....)
esp_err_t reinit_all_sensors(i2c_master_bus_handle_t i2c_handle) {

     esp_err_t ret = ESP_OK;

    //1. 상태 비트 끄기 (제어 루프 차단)
    g_system_health &= ~(SYS_HEALTH_IMU_OK | SYS_HEALTH_BARO_OK | SYS_HEALTH_MAG_OK);

    // 2. ICM20948 (IMU 1 & 2) 초기화
    //    가속도/자이로 범위, 샘플 레이트, LP 필터 등 설정
    imu_handle[0]=ICM20948::initialize(i2c_handle,ICM20948::ADDR_VCC);
    imu_handle[1]=ICM20948::initialize(i2c_handle,ICM20948::ADDR_GND);

    if (imu_handle[0] != NULL){
        g_system_health |= SYS_HEALTH_IMU_OK;
        ICM20948::enable_mag_bypass(imu_handle[0]);
    }
    else{
        ret = ESP_FAIL;
    }

    mag_handle[0] = IST8310::initialize(i2c_handle);
    mag_handle[1] = AK09916::initialize(i2c_handle);
    if (mag_handle[0] != NULL || mag_handle[1] != NULL){
        g_system_health |= SYS_HEALTH_MAG_OK;
    }else{
        ret = ESP_FAIL;
    }

    cbmp388_main.initialize(i2c_handle,CBMP388::ADDR_VCC);
    cbmp388_sub.initialize(i2c_handle,CBMP388::ADDR_GND);
    if(cbmp388_main.get_handle() != NULL || cbmp388_sub.get_handle() != NULL){
        g_system_health |= SYS_HEALTH_BARO_OK;
    }else{
        ret = ESP_FAIL;
    }
    
    ESP_LOGI("REINIT", "All sensors re-initialized. Health: 0x%08X", (unsigned int)g_system_health);
    return ret;
}


}
