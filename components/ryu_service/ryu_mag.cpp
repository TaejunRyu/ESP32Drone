#include "ryu_mag.h"

#include <esp_log.h>
#include "ryu_ist8310.h"
#include "ryu_ak09916.h"
#include "ryu_sensor_event.h"

namespace Service{
    
esp_err_t Service::ManageMag::initialize(){
    if (_initialized) return ESP_OK;
    auto& ist8310 = Sensor::IST8310::get_instance();
    auto& ak09916 = Sensor::AK09916::get_instance();
    if (!ist8310.is_initialized()){
        ist8310.initialize();
    }
    if (!ak09916.is_initialized()){
        ak09916.initialize();
    }
    _initialized = true;
    return ESP_OK;
}

std::tuple<esp_err_t, std::array<float, 3>> ManageMag::Managed_read_with_offset(){
    auto& ist8310 = Sensor::IST8310::get_instance();
    auto& ak09916 = Sensor::AK09916::get_instance();

    const float diff_x =  0.2784f;
    const float diff_y = -0.1175f;
    const float diff_z = -0.1285;

    static size_t active_index = 0;
    static size_t err_count = 0;        
    static size_t err_continue_count = 0;
    static float avr_mag[3] = {};
    static bool is_fault_posted = false; // 이벤트 중복 발행 방지
    esp_err_t err = ESP_OK;

    // [핵심] 두 센서 모두 임계치 초과 시 이벤트 발행
    if (err_continue_count > 6) {
        if (!is_fault_posted) {
            Service::fault_event_data_t data = { 
                .id = Service::FAULT_ID_MAG, 
                .is_recovered = false,
                .reason = ESP_ERR_TIMEOUT 
            };
            // Failsafe 모듈에게 "IMU 둘 다 먹통임"을 알림
            esp_event_post(Service::SYS_FAULT_EVENT_BASE, Service::SENSOR_EVENT_READ_FAILED, 
                           &data, sizeof(data), 0);
            is_fault_posted = true; 
            ESP_LOGE(TAG, "Both MAG sensors failed. Event posted.");
        }
        return {ESP_FAIL, {avr_mag[0], avr_mag[1], avr_mag[2]}}; 
    }

    // 센서 읽기 로직 (Main/Sub 스위칭)
    if (active_index == 0){
        auto [err_temp,mag] =  ist8310.read_with_offset();
        if (err_temp == ESP_OK)
            for(int ii=0; ii<3; ii++) { avr_mag[ii] = mag[ii]; }
        err = err_temp;
    }
    else if(active_index == 1){
        auto [err_temp,mag] = ak09916.read_with_offset();       
        if (err_temp == ESP_OK)
            for(int ii=0; ii<3; ii++) { avr_mag[ii] = mag[ii]; }
        avr_mag[0] = avr_mag[0] +  diff_x ;
        avr_mag[1] = avr_mag[1] +  diff_y ;
        avr_mag[2] = avr_mag[2] +  diff_z ;
        err = err_temp;
    }

    if (err == ESP_OK) {
        // 성공 시 데이터 업데이트 및 에러 카운트 초기화
        err_count = 0;
        err_continue_count = 0;

        // [핵심] 복구되었다면 복구 이벤트 발행
        if (is_fault_posted) {
            Service::fault_event_data_t data = { 
                .id = Service::FAULT_ID_MAG, 
                .is_recovered = true,
                .reason = ESP_ERR_TIMEOUT 
            };
            esp_event_post(Service::SYS_FAULT_EVENT_BASE, Service::SENSOR_EVENT_READ_RECOVERED, &data, sizeof(data), 0);
            is_fault_posted = false;
        }
        return {ESP_OK, {avr_mag[0],avr_mag[1],avr_mag[2]}};
    } 
    else {
        // 실패 시 스위칭 로직
        err_count++;
        if (err_count > 3) {
            ESP_LOGW(TAG, "Mag %d failed, switching...", active_index);
            active_index = (active_index == 0) ? 1 : 0; // 스위칭
            err_count = 0;
            err_continue_count++;
        }
        return {err, {avr_mag[0], avr_mag[1], avr_mag[2]}};
    }
} 
} // namespace Service