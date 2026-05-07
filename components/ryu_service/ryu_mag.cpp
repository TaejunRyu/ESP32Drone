#include "ryu_mag.h"

#include <esp_log.h>
#include "ryu_ist8310.h"
#include "ryu_ak09916.h"
#include "ryu_sensor_event.h"

namespace Service{
    
//ESP_EVENT_DEFINE_BASE(SYS_FAULT_EVENT_BASE);

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

    // 현재 bustype 어떤건지 알아야 ak09916의 읽기를 어떻게 처리할것인가 결정한다.
    // flight_task에서 i2c인가 spi인가를 알아야 imu,mag의 처리 방법 강구.
    _busType = ist8310.get_bus()->get_type();
    _initialized = true;
    return ESP_OK;
}


/**
 * @brief 
 *      1. 두개의 센서를 에러 복구를 위해서 가지고 있다가 문제가 발생하면 바로 보조 센서로 제어권을 넘김.
 *      2. 지자계센서의 문제는 에러 복구를 하지 않고 바로 GROUND STATION에 알려 조치를 하도록 한다.( 이게 좋은 방법일지도....)
 * @return std::tuple<esp_err_t, std::array<float, 3>> 
 */
std::tuple<esp_err_t, std::array<float, 3>> ManageMag::Managed_read_with_offset(){
    auto& ist8310 = Sensor::IST8310::get_instance();
    auto& ak09916 = Sensor::AK09916::get_instance();

    esp_err_t err = ESP_OK;
    // [핵심] 두 센서 모두 임계치 초과 시 이벤트 발행
    if (this->err_continue_count > 6) {
        if (!this->is_fault_posted) {
            Service::fault_event_data_t data = { 
                .id = Service::FAULT_ID_MAG, 
                .is_recovered = false,
                .reason = ESP_ERR_TIMEOUT 
            };
            // Failsafe 모듈에게 "MAG 둘 다 먹통임"을 알림
            esp_event_post(Service::SYS_FAULT_EVENT_BASE, Service::SENSOR_EVENT_READ_FAILED, &data, sizeof(data), 0);
            
            // 바로 풀어버리면 될까 ???????????????????????????????
            this->is_fault_posted = true; 
            ESP_LOGE(TAG, "Both MAG sensors failed. Event posted.");
        }
        return {ESP_FAIL, {this->avr_mag[0], this->avr_mag[1], this->avr_mag[2]}}; 
    }

    // 센서 읽기 로직 (Main/Sub 스위칭)
    if (this->active_index == 0){
        auto [err_temp,mag] =  ist8310.read_with_offset();
        if (err_temp == ESP_OK)
            for(int ii=0; ii<3; ii++) { this->avr_mag[ii] = mag[ii]; }
        err = err_temp;
    }
    else if(this->active_index == 1){
        auto [err_temp,mag] = ak09916.read_with_offset();       
        if (err_temp == ESP_OK)
            for(int ii=0; ii<3; ii++) { this->avr_mag[ii] = mag[ii]; }
        this->avr_mag[0] = this->avr_mag[0] +  diff_x ;
        this->avr_mag[1] = this->avr_mag[1] +  diff_y ;
        this->avr_mag[2] = this->avr_mag[2] +  diff_z ;
        err = err_temp;
    }

    if (err == ESP_OK) {
        // 성공 시 데이터 업데이트 및 에러 카운트 초기화
        this->err_count = 0;
        this->err_continue_count = 0;

        // [핵심] 복구되었다면 복구 이벤트 발행
        if (this->is_fault_posted) {
            Service::fault_event_data_t data = { 
                .id = Service::FAULT_ID_MAG, 
                .is_recovered = true,
                .reason = ESP_ERR_TIMEOUT
            };
            esp_event_post(Service::SYS_FAULT_EVENT_BASE, Service::SENSOR_EVENT_READ_RECOVERED, &data, sizeof(data), 0);
            this->is_fault_posted = false;
        }
        return {ESP_OK, {this->avr_mag[0],this->avr_mag[1],this->avr_mag[2]}};
    } 
    else {
        // 실패 시 스위칭 로직
        this->err_count++;
        if (this->err_count > 3) {
            ESP_LOGW(TAG, "Mag %d failed, switching...", this->active_index);
            this->active_index = (this->active_index == 0) ? 1 : 0; // 스위칭
            this->err_count = 0;
            this->err_continue_count++;
        }
        return {err, {this->avr_mag[0], this->avr_mag[1], this->avr_mag[2]}};
    }
} 
} // namespace Service