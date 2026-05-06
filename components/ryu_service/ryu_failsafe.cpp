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

esp_err_t FailSafe::initialize() {
    if(_initialized) return ESP_OK;

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
            g_sys.error_hold_mode = true;  // 에러가 발생하여 HOLD_MODE 상태로 전환 ( 이 곳에서도 시스템 모드전환 및 산태 프래그를 체크하여 현재모드 설정을 어떻게 할지)
            break;
        }
        case FAULT_ID_BARO:{
            bit = SYS_HEALTH_BARO_OK;
            break;
        }
        case FAULT_ID_MAG:  bit = SYS_HEALTH_MAG_OK; break;
        case FAULT_ID_GPS:{  
            bit = SYS_HEALTH_GPS_OK; 
            g_sys.error_hold_mode = true;
            break;
        }
        case FAULT_ID_RC:{   
            bit = SYS_HEALTH_RC_OK; 
            g_sys.error_hold_mode = true;
            break;
        }
        default: break;
    }

    if (fault->is_recovered) {
        system_health |= bit;  // 비트 셋 (정상)
        ESP_LOGI(TAG, "Sensor %d Recovered. Health: 0x%08X", fault->id, system_health);
    } else {
        system_health &= ~bit; // 비트 클리어 (에러)
        ESP_LOGE(TAG, "Sensor %d Fault! Reason: %s", fault->id, esp_err_to_name(fault->reason));
        
        // 특정 비트 조합에 따른 즉각 대응 로직 수행 가능
        if (!(system_health & MASK_REQUIRED_MANUAL)) { //IMU와 RC가 정상이 아니면...
            ESP_LOGW(TAG, "CRITICAL: Manual flight impossible!");
        }
    }
}


// 매니저 태스크에서는 system_health를 주기적으로 체크하며 전체 복구 로직 실행
void FailSafe::failsafe_manager_task(void *pvParameters) {
    FailSafe& instance = FailSafe::get_instance();
    while(1) {
        if (!(instance.system_health & SYS_HEALTH_IMU_OK)) { //IMU가 정상이 아니면 센서 전체 initialize...
            // IMU가 죽어있다면 여기서 I2C 리셋 시도 등 전역 복구 수행    
            instance.reinit_all_sensors();
            g_sys.error_hold_mode = false;
        }        
        if (!(instance.system_health & SYS_HEALTH_BARO_OK)) { 
            // baro sensor check   ,  수동 전환하여 Go to Home         
        }
        if (!(instance.system_health & SYS_HEALTH_MAG_OK)) { 
            // mag sensor check    ,  수동 전환하여 Go to Home
        }
        if (!(instance.system_health & SYS_HEALTH_GPS_OK)) {             
            // gps sensor check    , 수동 전환하여 Go to Home

            // 정상작동후 처리 
            g_sys.error_hold_mode = false;
        }
        if (!(instance.system_health & SYS_HEALTH_RC_OK)) { 
            // flysky sensor check , QGC든지 아니면 BRIGE에서 신호를           

            // 정상작동후 처리
            g_sys.error_hold_mode = false;
        }
        if (!(instance.system_health & SYS_HEALTH_BATTERY_OK)) { 
            // battery sensor check , RTL 처리            
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t FailSafe::reinit_all_sensors()
{
    esp_err_t ret = ESP_OK;
    //1. 상태 비트 끄기 (제어 루프 차단)
    system_health &= ~(SYS_HEALTH_IMU_OK | SYS_HEALTH_BARO_OK | SYS_HEALTH_MAG_OK);

    Sensor::AK09916::get_instance().deinitialize();
    Sensor::ICM20948::Main().deinitialize();
    Sensor::ICM20948::Sub().deinitialize();
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
    Sensor::ICM20948::Main().setup_i2c_interface(Driver::I2C::get_instance().get_bus_handle(),Sensor::ICM20948::ADDR_VCC);
    Sensor::ICM20948::Main().initialize();
    Sensor::ICM20948::Sub().setup_i2c_interface(Driver::I2C::get_instance().get_bus_handle(),Sensor::ICM20948::ADDR_GND);
    Sensor::ICM20948::Sub().initialize();
    if (Sensor::ICM20948::Main().is_initialized()){
        system_health |= SYS_HEALTH_IMU_OK;
        Sensor::ICM20948::Main().enable_mag_bypass();
    }
    else{
        ret = ESP_FAIL;
    }
    Sensor::IST8310::get_instance().initialize();    
    Sensor::AK09916::get_instance().initialize();
    if ( Sensor::IST8310::get_instance().is_initialized()){
        system_health |= SYS_HEALTH_MAG_OK;
    }else{
        ret = ESP_FAIL;
    }
    Sensor::BMP388::Main().initialize();
    Sensor::BMP388::Sub().initialize();
    if(Sensor::BMP388::Main().is_initialized()){
        system_health |= SYS_HEALTH_BARO_OK;
    }else{
        ret = ESP_FAIL;
    }
    ESP_LOGI(TAG, "All sensors re-initialized. Health: 0x%08X", (unsigned int)system_health);
    return ret;
}

BaseType_t FailSafe::start_task()
{
    auto res= xTaskCreatePinnedToCore(failsafe_manager_task, "failsafe_manager_task", 4096, this, 10,&_task_handle, 0);
    if (res != pdPASS) ESP_LOGE(TAG, "❌ 5.FailSafe Task is failed! code: %d", res);
    else ESP_LOGI(TAG, "✓ 5.FailSafe Task is passed...");
    return res;
}

} // namespace Service
