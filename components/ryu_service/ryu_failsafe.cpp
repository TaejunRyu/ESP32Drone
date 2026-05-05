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
#include "esp_log.h"


namespace Service {

ESP_EVENT_DEFINE_BASE(SYS_FAULT_EVENT_BASE);

const char* FailSafe::TAG = "FailSafe";

FailSafe::FailSafe(){ 
    system_health = SYS_HEALTH_ALL_OK; // 초기 상태 설정
    ESP_LOGI(TAG,"Initializing FailSafe Service.");
}

FailSafe::~FailSafe(){}

esp_err_t FailSafe::initialize() {
    if(_initialized) return ESP_OK;

    // 기본 이벤트 루프 생성 (이미 생성되어 있다면 생략 가능)
    // esp_err_t err = esp_event_loop_create_default();
    // if (err != ESP_OK){
    //     ESP_LOGI(TAG,"ESP_EVENT_LOOP creation failed.");
    //     return err;
    // }
    // 이벤트 핸들러 등록: 어떤 센서든 에러가 나면 이 핸들러로 모임
    esp_err_t err = esp_event_handler_instance_register(SYS_FAULT_EVENT_BASE, ESP_EVENT_ANY_ID,
                                               &FailSafe::event_handler_relay, this, NULL);
    if (err != ESP_OK){
        ESP_LOGI(TAG,"esp_event_handler_instance_register failed.");
        return err;
    }
    _initialized = true;
    ESP_LOGI(TAG,"Initialized successfully.");
    
    return err; 
}

// 릴레이 함수 (C 스타일 콜백을 클래스 메서드로 연결)
void FailSafe::event_handler_relay(void* arg, esp_event_base_t base, int32_t id, void* data) {
    auto* obj = static_cast<FailSafe*>(arg);
    auto* fault = static_cast<fault_event_data_t*>(data);
    obj->update_health(fault);
}

// event_handler_relay에서 실행하여 이벤트 올때마다 비트 변경.
void FailSafe::update_health(fault_event_data_t* fault) {
    uint32_t bit = 0;
    
    // ID에 매칭되는 비트 결정
    switch(fault->id) {
        case FAULT_ID_IMU:{  
            bit = SYS_HEALTH_IMU_OK;
            ESP_LOGI(TAG,"EVENT...."); 
            break;
        }
        case FAULT_ID_MAG:  bit = SYS_HEALTH_MAG_OK; break;
        case FAULT_ID_BARO: bit = SYS_HEALTH_BARO_OK; break;
        case FAULT_ID_GPS:  bit = SYS_HEALTH_GPS_OK; break;
        case FAULT_ID_RC:   bit = SYS_HEALTH_RC_OK; break;
        default: break;
    }

    if (fault->is_recovered) {
        system_health |= bit;  // 비트 셋 (정상)
        ESP_LOGI(TAG, "Sensor %d Recovered. Health: 0x%08X", fault->id, system_health);
    } else {
        system_health &= ~bit; // 비트 클리어 (에러)
        ESP_LOGE(TAG, "Sensor %d Fault! Reason: %s", fault->id, esp_err_to_name(fault->reason));
        
        // 특정 비트 조합에 따른 즉각 대응 로직 수행 가능
        if (!(system_health & MASK_REQUIRED_MANUAL)) {
            ESP_LOGW(TAG, "CRITICAL: Manual flight impossible!");
        }
    }
}

esp_err_t FailSafe::reinit_all_sensors()
{
    esp_err_t ret = ESP_OK;
    //1. 상태 비트 끄기 (제어 루프 차단)
    system_health &= ~(SYS_HEALTH_IMU_OK | SYS_HEALTH_BARO_OK | SYS_HEALTH_MAG_OK);

    Sensor::ICM20948::Main().deinitialize();
    Sensor::ICM20948::Sub().deinitialize();
    Sensor::AK09916::get_instance().deinitialize();
    Sensor::IST8310::get_instance().deinitialize();
    Sensor::BMP388::Main().deinitialize();
    Sensor::BMP388::Sub().deinitialize();
    Driver::I2C::get_instance().deinitialize();

        // 2. SCL/SDA 핀 제어권 획득 및 강제 토글 (Bus Clear)
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << Driver::I2C::I2C_SCL) | (1ULL << Driver::I2C::I2C_SDA);
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD; 
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // SCL 9번 토글 (슬레이브 장치 리셋 유도)
    for (int i = 0; i < 9; i++) {
        gpio_set_level(Driver::I2C::I2C_SCL, 0);
        esp_rom_delay_us(5);
        gpio_set_level(Driver::I2C::I2C_SCL, 1);
        esp_rom_delay_us(5);
    }

    Driver::I2C::get_instance().initialize();

    // 2. ICM20948 (IMU 1 & 2) 초기화
    //    가속도/자이로 범위, 샘플 레이트, LP 필터 등 설정
    Sensor::ICM20948::Main().initialize();
    Sensor::ICM20948::Sub().initialize();

    if (Sensor::ICM20948::Main().get_dev_handle() != NULL){
        system_health |= SYS_HEALTH_IMU_OK;
        Sensor::ICM20948::Main().enable_mag_bypass();
    }
    else{
        ret = ESP_FAIL;
    }

    Sensor::IST8310::get_instance().initialize();    
    Sensor::AK09916::get_instance().initialize();

    if ( Sensor::IST8310::get_instance().get_dev_handle() != nullptr ){
        system_health |= SYS_HEALTH_MAG_OK;
    }else{
        ret = ESP_FAIL;
    }

    Sensor::BMP388::Main().initialize();
    Sensor::BMP388::Sub().initialize();

    if(Sensor::BMP388::Main().get_dev_handle() != NULL){
        system_health |= SYS_HEALTH_BARO_OK;
    }else{
        ret = ESP_FAIL;
    }
    ESP_LOGI("REINIT", "All sensors re-initialized. Health: 0x%08X", (unsigned int)system_health);
    return ret;
}


// 매니저 태스크에서는 system_health를 주기적으로 체크하며 전체 복구 로직 실행
void FailSafe::failsafe_manager_task(void *pvParameters) {
    FailSafe& instance = FailSafe::get_instance();
    while(1) {
        if (!(instance.system_health & SYS_HEALTH_IMU_OK)) {
            // IMU가 죽어있다면 여기서 I2C 리셋 시도 등 전역 복구 수행
            ESP_LOGI(TAG,"이벤트발생!!!!!!!!!!!!!!!!!!!");
            instance.reinit_all_sensors();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


void FailSafe::start_task()
{
    auto res= xTaskCreatePinnedToCore(failsafe_manager_task, "failsafe_manager_task", 4096, this, 10,&_task_handle, 0);
    if (res != pdPASS) ESP_LOGE(TAG, "❌ 5.Error Check Task is failed! code: %d", res);
    else ESP_LOGI(TAG, "✓ 5.Error Check Task is passed...");
}


}

// namespace Service{


// const char* FailSafe::TAG = "FailSafe";

// FailSafe::FailSafe(){
//     ESP_LOGI(TAG,"Initializing FailSafe Service...");
// }

// FailSafe::~FailSafe(){}

// esp_err_t FailSafe::initialize()
// {
//     if(_initialized) return ESP_OK;
//     //
//     _initialized = true;
//     ESP_LOGI(TAG,"Initialized successfully.");
//     return ESP_OK;
// }

// esp_err_t FailSafe::reinit_all_sensors()
// {
//     esp_err_t ret = ESP_OK;

//     //i2c_master_bus_handle_t i2c_handle = Driver::I2C::get_instance().get_bus_handle();

//     //1. 상태 비트 끄기 (제어 루프 차단)
//     system_health &= ~(SYS_HEALTH_IMU_OK | SYS_HEALTH_BARO_OK | SYS_HEALTH_MAG_OK);

//     // 2. ICM20948 (IMU 1 & 2) 초기화
//     //    가속도/자이로 범위, 샘플 레이트, LP 필터 등 설정
//     Sensor::ICM20948::Main().initialize();
//     Sensor::ICM20948::Sub().initialize();

//     if (Sensor::ICM20948::Main().get_dev_handle() != NULL){
//         system_health |= SYS_HEALTH_IMU_OK;
//         Sensor::ICM20948::Main().enable_mag_bypass();
//     }
//     else{
//         ret = ESP_FAIL;
//     }
//     Sensor::IST8310::get_instance().deinitialize();
//     Sensor::IST8310::get_instance().initialize();
    
//     Sensor::AK09916::get_instance().deinitialize();
//     Sensor::AK09916::get_instance().initialize();

//     if ( Sensor::IST8310::get_instance().get_dev_handle() != NULL || 
//          Sensor::AK09916::get_instance().get_dev_handle() != NULL){
//         system_health |= SYS_HEALTH_MAG_OK;
//     }else{
//         ret = ESP_FAIL;
//     }

//     // deinitialize 해야할것 같음....

//     Sensor::BMP388::Main().initialize();
//     Sensor::BMP388::Sub().initialize();

//     if(Sensor::BMP388::Main().get_dev_handle() != NULL || Sensor::BMP388::Sub().get_dev_handle() != NULL){
//         system_health |= SYS_HEALTH_BARO_OK;
//     }else{
//         ret = ESP_FAIL;
//     }
    
//     ESP_LOGI("REINIT", "All sensors re-initialized. Health: 0x%08X", (unsigned int)system_health);
//     return ret;
// }


// void FailSafe::failsafe_manager_task(void * pvParameters)
// {
//     auto failsafe = static_cast<FailSafe*>(pvParameters);
//     uint32_t notifiedValue;
//     esp_err_t ret=ESP_FAIL;
//     while (true) {
//         // 알림이 올 때까지 무한 대기 (CPU 점유율 0%)
//         if (xTaskNotifyWait(0x00, ULONG_MAX, &notifiedValue, portMAX_DELAY) == pdPASS) {
//             //  IMU/MAG/BARO  모두 에러 발생 또는 I2C 에러시 모든 것 리셋.
//             if (notifiedValue & ERR_I2C_BUS_HANG) {
//                 failsafe->system_health &= ~SYS_HEALTH_IMU_OK;  // 상태 차단                     
//                 Driver::I2C::get_instance().deinitialize(); // 하드웨어 리셋 (SCL 토글)
//                 Driver::I2C::get_instance().initialize();
//                 if (Driver::I2C::get_instance().get_bus_handle() !=NULL){ 
//                     ret = failsafe->reinit_all_sensors();             // 센서 레지스터 재설정
//                     auto [ret_code,macc,mgyro] =Sensor::ICM20948::Main().read_raw_data();
//                     if (ret_code == ESP_OK && (macc[0] + macc[1] + macc[2]) > 1.0f){
//                         ret = ESP_OK;
//                         auto& flight = Controller::Flight::get_instance();    
//                         flight.imu_error_cnt =0;
//                         flight.mag_error_cnt =0;
//                         flight.baro_error_cnt =0;
//                         flight.imu_active_index = 0;
//                         flight. mag_active_index = 0;
//                         flight.baro_active_index = 0;
//                         g_sys.error_hold_mode = false;                
//                         // 모든 센서가 정상으로 돌아왔다고 가정하고 상태 비트 모두 켜기 (필요에 따라 개별적으로 설정할 수도 있음)
//                         failsafe->system_health |= SYS_HEALTH_IMU_OK|SYS_HEALTH_MAG_OK|SYS_HEALTH_BARO_OK;                    
//                     }
//                 }else{
//                     ret = ESP_FAIL;
//                 }
//                 if (ret != ESP_OK){
//                     Driver::Motor::get_instance().stop_all_motors();
//                     while(true){
//                         Driver::Buzzer::get_instance().sound_emergency(); // 나 죽었다~~~~~~~~
//                     }
//                 }
//                 printf("⚠️ WARNING: IMU/MAG/BARO Not Working Detected!\n");
//             }

//             // 2. 조종기 신호 상실 처리
//             if (notifiedValue & ERR_RC_LOST) {
//                 Driver::Buzzer::get_instance().sound_emergency();
//                 g_sys.error_hold_mode = false;                
//                 // Telemetry로 경고 전송 및 로그 저장 로직
//                 //save_error_to_nvs("RC_LOST");
//                 printf("⚠️ WARNING: Lost Control Signal Detected!\n");
//             }

//             // 3. 배터리 저전압 처리
//             if (notifiedValue & ERR_BATTERY_LOW) {
//                 Driver::Buzzer::get_instance().sound_low_battery();
//                 printf("⚠️ WARNING: Low Battery Detected!\n");
//             }
            
//             // 4. GPS TIMEOUT
//             if (notifiedValue & ERR_GPS_TIMEOUT) {
//                 //Driver::sound_emergency();
//                 g_sys.error_hold_mode = false;                
//                 printf(" {(%lld): ⚠️ WARNING: Gps Timeout Detected!\n",esp_timer_get_time());
//             }

//             // 처리 완료 후 로그 출력
//             //std::printf("Error Resolved: 0x%08x\n", (unsigned int)notifiedValue);
//         }
//     }
// }

// void FailSafe::start_task()
// {
//     auto res= xTaskCreatePinnedToCore(failsafe_manager_task, "failsafe_manager_task", 4096, this, 10,&_task_handle, 0);
//     if (res != pdPASS) ESP_LOGE(TAG, "❌ 5.Error Check Task is failed! code: %d", res);
//     else ESP_LOGI(TAG, "✓ 5.Error Check Task is passed...");
// }
// }
