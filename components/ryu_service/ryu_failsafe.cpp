#include "ryu_failsafe.h"

#include <esp_timer.h>
#include "ryu_i2c.h"
#include "ryu_icm20948.h"
#include "ryu_ak09916.h"
#include "ryu_bmp388.h"
#include "ryu_ist8310.h"
#include "ryu_motor.h"
#include "ryu_buzzer.h"
#include "ryu_flight_task.h"
#include "ryu_config.h"

namespace Service{


const char* FailSafe::TAG = "FailSafe";

FailSafe::FailSafe(){
    ESP_LOGI(TAG,"Initializing FailSafe Service...");
}

FailSafe::~FailSafe(){}

esp_err_t FailSafe::initialize()
{
    if(_initialized) return ESP_OK;
    //
    _initialized = true;
    ESP_LOGI(TAG,"Initialized successfully.");
    return ESP_OK;
}

esp_err_t FailSafe::reinit_all_sensors()
{
    esp_err_t ret = ESP_OK;

    //i2c_master_bus_handle_t i2c_handle = Driver::I2C::get_instance().get_bus_handle();

    //1. 상태 비트 끄기 (제어 루프 차단)
    g_system_health &= ~(SYS_HEALTH_IMU_OK | SYS_HEALTH_BARO_OK | SYS_HEALTH_MAG_OK);

    // 2. ICM20948 (IMU 1 & 2) 초기화
    //    가속도/자이로 범위, 샘플 레이트, LP 필터 등 설정
    Sensor::ICM20948::Main().initialize();
    Sensor::ICM20948::Sub().initialize();

    if (Sensor::ICM20948::Main().get_dev_handle() != NULL){
        g_system_health |= SYS_HEALTH_IMU_OK;
        Sensor::ICM20948::Main().enable_mag_bypass();
    }
    else{
        ret = ESP_FAIL;
    }
    Sensor::IST8310::get_instance().deinitialize();
    Sensor::IST8310::get_instance().initialize();
    
    Sensor::AK09916::get_instance().deinitialize();
    Sensor::AK09916::get_instance().initialize();

    if ( Sensor::IST8310::get_instance().get_dev_handle() != NULL || 
         Sensor::AK09916::get_instance().get_dev_handle() != NULL){
        g_system_health |= SYS_HEALTH_MAG_OK;
    }else{
        ret = ESP_FAIL;
    }

    // deinitialize 해야할것 같음....

    Sensor::BMP388::Main().initialize();
    Sensor::BMP388::Sub().initialize();

    if(Sensor::BMP388::Main().get_dev_handle() != NULL || Sensor::BMP388::Sub().get_dev_handle() != NULL){
        g_system_health |= SYS_HEALTH_BARO_OK;
    }else{
        ret = ESP_FAIL;
    }
    
    ESP_LOGI("REINIT", "All sensors re-initialized. Health: 0x%08X", (unsigned int)g_system_health);
    return ret;
}


void FailSafe::failsafe_manager_task(void * pvParameters)
{
    auto failsafe = static_cast<FailSafe*>(pvParameters);
    uint32_t notifiedValue;
    esp_err_t ret=ESP_FAIL;

    while (true) {
        // 알림이 올 때까지 무한 대기 (CPU 점유율 0%)
        if (xTaskNotifyWait(0x00, ULONG_MAX, &notifiedValue, portMAX_DELAY) == pdPASS) {

            //  IMU/MAG/BARO  모두 에러 발생 또는 I2C 에러시 모든 것 리셋.
            if (notifiedValue & ERR_I2C_BUS_HANG) {
                failsafe->g_system_health &= ~SYS_HEALTH_IMU_OK;  // 상태 차단                                
                Driver::I2C::get_instance().deinitialize(); // 하드웨어 리셋 (SCL 토글)        
                
                Driver::I2C::get_instance().initialize();
                if (Driver::I2C::get_instance().get_bus_handle() !=NULL){ 
                    ret = failsafe->reinit_all_sensors();             // 센서 레지스터 재설정
                    auto [ret_code,macc,mgyro] =Sensor::ICM20948::Main().read_raw_data();
                    if (ret_code == ESP_OK && (macc[0] + macc[1] + macc[2]) > 1.0f){
                        ret = ESP_OK;
                        auto& flight = Controller::Flight::get_instance();    
                        flight.imu_error_cnt =0;
                        flight.mag_error_cnt =0;
                        flight.baro_error_cnt =0;
                        flight.imu_active_index = 0;
                        flight.mag_active_index = 0;
                        flight.baro_active_index = 0;

                        g_sys.error_hold_mode = false;                
                        
                        // 모든 센서가 정상으로 돌아왔다고 가정하고 상태 비트 모두 켜기 (필요에 따라 개별적으로 설정할 수도 있음)
                        failsafe->g_system_health |= SYS_HEALTH_IMU_OK|SYS_HEALTH_MAG_OK|SYS_HEALTH_BARO_OK;                    
                    }
                }else{
                    ret = ESP_FAIL;
                }

                if (ret != ESP_OK){
                    Driver::Motor::get_instance().stop_all_motors();
                    while(true){
                        Driver::Buzzer::get_instance().sound_emergency(); // 나 죽었다~~~~~~~~
                    }
                }
                printf("⚠️ WARNING: IMU/MAG/BARO Not Working Detected!\n");
            }

            // 2. 조종기 신호 상실 처리
            if (notifiedValue & ERR_RC_LOST) {
                Driver::Buzzer::get_instance().sound_emergency();
                g_sys.error_hold_mode = false;                
                // Telemetry로 경고 전송 및 로그 저장 로직
                //save_error_to_nvs("RC_LOST");
                printf("⚠️ WARNING: Lost Control Signal Detected!\n");
            }

            // 3. 배터리 저전압 처리
            if (notifiedValue & ERR_BATTERY_LOW) {
                Driver::Buzzer::get_instance().sound_low_battery();
                printf("⚠️ WARNING: Low Battery Detected!\n");
            }
            
            // 4. GPS TIMEOUT
            if (notifiedValue & ERR_GPS_TIMEOUT) {
                //Driver::sound_emergency();
                g_sys.error_hold_mode = false;                
                printf(" {(%lld): ⚠️ WARNING: Gps Timeout Detected!\n",esp_timer_get_time());
            }

            // 처리 완료 후 로그 출력
            //std::printf("Error Resolved: 0x%08x\n", (unsigned int)notifiedValue);
        }
    }
}

void FailSafe::start_task()
{
    auto res= xTaskCreatePinnedToCore(failsafe_manager_task, "failsafe_manager_task", 4096, this, 10,&_task_handle, 0);
    if (res != pdPASS) ESP_LOGE(TAG, "❌ 5.Error Check Task is failed! code: %d", res);
    else ESP_LOGI(TAG, "✓ 5.Error Check Task is passed...");
}
}
