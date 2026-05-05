#include "ryu_icm20948.h"

#include <tuple>
#include <freertos/FreeRTOS.h>
#include <esp_timer.h>
#include "ryu_i2c.h"
#include "ryu_sensor_event.h"

namespace Sensor{

const char *ICM20948::TAG = "ICM20948";

ESP_EVENT_DEFINE_BASE(SYS_FAULT_EVENT_BASE);

ICM20948 ICM20948::mainInstance("ICM20948 Main",ICM20948::ADDR_VCC);
ICM20948 ICM20948::subInstance("ICM20948 Sub",ICM20948::ADDR_GND);

void ICM20948::icm20948_select_bank(uint8_t bank)
{
    uint8_t cmd[] = {REG_BANK_SEL, (uint8_t)(bank << 4)};
    i2c_master_transmit(_dev_handle, cmd, 2, pdMS_TO_TICKS(10));
    _current_bank = bank;
}

esp_err_t ICM20948::initialize()
{
    esp_err_t err;
    if(_initialized && _dev_handle) {
        ESP_LOGI(TAG,"%s Already initialized.",_name.c_str());
        return ESP_OK;
    }
    // bus handle을 바로 가져오기
    if (Driver::I2C::get_instance().get_bus_handle() != nullptr){
        this->_bus_handle = Driver::I2C::get_instance().get_bus_handle();
    }
    else{
        ESP_LOGE(TAG, "%s : Failed to get i2c handle", _name.c_str());
        return ESP_FAIL;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = this->_dev_address;
    dev_cfg.scl_speed_hz = Driver::I2C::I2C_SPEED;; // 400kHz

    // 버스에 장치 추가 및 핸들 획득
    err = i2c_master_bus_add_device(_bus_handle, &dev_cfg, &_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG,"%s Failed to add device to bus.",_name.c_str());
        this->_isAlive = false; // 명시적으로 상태 기록
        return err;
    }

    icm20948_select_bank(0);
    
    // 1. Soft Reset
    uint8_t reset_cmd[] = {B0_PWR_MGMT_1, 0x80};
    err = i2c_master_transmit(_dev_handle, reset_cmd, 2, pdMS_TO_TICKS(100));
    if(err != ESP_OK){
        ESP_LOGE(TAG, "%s : Soft Reset setting failed", _name.c_str());
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // 리셋 후 대기 필수

    // 2. Sleep 해제 및 Auto Clock 선택 (내부 20MHz OSC 사용)
    uint8_t wake_cmd[] = {B0_PWR_MGMT_1, 0x01};
    err = i2c_master_transmit(_dev_handle, wake_cmd, 2, pdMS_TO_TICKS(100));
    if(err != ESP_OK){
        ESP_LOGE(TAG, "%s : Sleep & Auto Clock setting failed", _name.c_str());
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // --- [1. Bank 2로 이동] ---
    icm20948_select_bank(2);
    vTaskDelay(pdMS_TO_TICKS(10));

    // --- [2. 자이로스코프 설정: ±1000dps & DLPF 설정] ---
    /* 
       GYRO_CONFIG_1 (0x01) 설정값 계산:
       - Bit [5:3]: DLPF 대역폭 설정 (3 = 24Hz, 4 = 12Hz) -> 400Hz 루프엔 '3'(24Hz)이 적당
       - Bit [2:1]: Full Scale (2 = ±1000dps)
       - Bit [0]: DLPF 활성화 (1 = Enable)
       => 0b00011101 (0x1D)
    */
    uint8_t gyro_cfg[] = {B2_GYRO_CONFIG_1, 0x1D};
    err = i2c_master_transmit(_dev_handle, gyro_cfg, 2, pdMS_TO_TICKS(100));
    if(err != ESP_OK){
        ESP_LOGE(TAG, "%s : Gyro DLPF setting failed", _name.c_str());
        return err;
    }

    // --- [3. 가속도계 설정: ±8g & DLPF 설정] ---
    /* 
       ACCEL_CONFIG (0x14) 설정값 계산:
       - Bit [5:3]: DLPF 대역폭 설정 (3 = 24.6Hz)
       - Bit [2:1]: Full Scale (2 = ±8g)
       - Bit [0]: DLPF 활성화 (1 = Enable)
       => 0b00011101 (0x1D)
    */
    uint8_t accel_cfg[] = {B2_ACCEL_CONFIG, 0x1D};
    err = i2c_master_transmit(_dev_handle, accel_cfg, 2, pdMS_TO_TICKS(100));
    if(err != ESP_OK){
        ESP_LOGE(TAG, "%s : Accel DLPF setting failed", _name.c_str());
        return err;
    }

    // --- [4. 다시 Bank 0로 복귀 (데이터 읽기 준비)] ---
    icm20948_select_bank(0);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    _initialized = true;
    _isAlive = true;
    ESP_LOGI(TAG,"%s Initialized sucessfully.",this->_name.c_str());    
    return err;
}

esp_err_t ICM20948::deinitialize()
{
    _initialized = false;
    _isAlive = false;
    return ESP_OK;
}

std::tuple<esp_err_t, std::array<float, 3>, std::array<float, 3>> ICM20948::read_raw_data()
{
    if (_dev_handle == NULL){ 
        return {ESP_FAIL, {0.0f, 0.0f, 0.0f},{0.0f,0.0f,0.0f}};
    }
    uint8_t d[12]; // Accel(6) + Gyro(6)
    
    icm20948_select_bank(0);
    esp_err_t ret = i2c_master_transmit_receive(_dev_handle, &B0_ACCEL_XOUT_H, 1, d, 12, pdMS_TO_TICKS(3));
    if(ret != ESP_OK){
        return{ret,{},{}};
    } 
    float raw_acc[3] = {},raw_gyro[3] ={};
    // Raw 데이터 결합 (Big-Endian)
    raw_acc[0]  = (int16_t)(d[0] << 8  | d[1])     / 4096.0f ;
    raw_acc[1]  = (int16_t)(d[2] << 8  | d[3])     / 4096.0f ;
    raw_acc[2]  = (int16_t)(d[4] << 8  | d[5])     / 4096.0f ;
    raw_gyro[0] = (int16_t)(d[6] << 8  | d[7])     / 32.8f   ;
    raw_gyro[1] = (int16_t)(d[8] << 8  | d[9])     / 32.8f   ;
    raw_gyro[2] = (int16_t)(d[10] << 8 | d[11])    / 32.8f   ;
    
    return {ret,{raw_acc[0],raw_acc[1],raw_acc[2]},{raw_gyro[0],raw_gyro[1],raw_gyro[2]}};
}

void ICM20948::calibrate()
{
    if (_calibration){
        ESP_LOGW(TAG,"%s Already Calibration was performed.",_name.c_str());
        return ;
    }
    ESP_LOGI(TAG, "%s : Zeroing... Keep the aircraft level.",_name.c_str());

    float sum_acc[3] ={},sum_gyro[3]={};
     int valid_samples = 0; // 실제로 성공한 샘플 수만 카운트
    const int samples = 500; // 500번 샘플링    
    uint8_t buf[12];
    uint8_t reg = B0_ACCEL_XOUT_H;
    
    icm20948_select_bank(0);
    for (int i = 0; i < samples; i++) {
        uint64_t start_time = esp_timer_get_time(); // 시작 시간 기록
        if (i2c_master_transmit_receive(_dev_handle, &reg, 1, buf, 12, pdMS_TO_TICKS(100)) == ESP_OK) [[likely]]{    
            sum_acc[0] += (int16_t)((buf[0] << 8) | buf[1])   / 4096.0f;
            sum_acc[1] += (int16_t)((buf[2] << 8) | buf[3])   / 4096.0f;
            sum_acc[2] += (int16_t)((buf[4] << 8) | buf[5])   / 4096.0f;
            
            sum_gyro[0] += (int16_t)((buf[6] << 8) | buf[7])   / 32.8f;  
            sum_gyro[1] += (int16_t)((buf[8] << 8) | buf[9])   / 32.8f;
            sum_gyro[2] += (int16_t)((buf[10] << 8) | buf[11]) / 32.8f;
            valid_samples++; 
        }  
        // 400Hz(2500us) 주기를 정확히 맞추기 위한 대기
        uint64_t end_time = esp_timer_get_time();
        uint32_t elapsed = (uint32_t)(end_time - start_time);
        if (elapsed < 2500) {
            esp_rom_delay_us(2500 - elapsed); // 남은 시간만큼 마이크로초 단위 대기
        }   
    }
    if(valid_samples > 0 )[[likely]]{
        // 평균값 계산 (Offset 저장)
        sum_acc[0]  = sum_acc[0] / valid_samples;
        sum_acc[1]  = sum_acc[1] / valid_samples;
        sum_acc[2]  = (sum_acc[2] / valid_samples) -1.0f;  // 가속도는 중력 가속도(1g)를 제외하고 0이 되어야 함m

        sum_gyro[0] = sum_gyro[0] / valid_samples;
        sum_gyro[1] = sum_gyro[1] / valid_samples;
        sum_gyro[2] = sum_gyro[2] / valid_samples;
    }else{
        sum_acc[0] =0.0f;sum_acc[1] =0.0f;sum_acc[2] =0.0f; 
        sum_gyro[0] = 0.0f,sum_gyro[1] = 0.0f,sum_gyro[2] = 0.0f;
    }
    
    _offset_acc[0] = sum_acc[0];
    _offset_acc[1] = sum_acc[1];
    _offset_acc[2] = sum_acc[2];
    _offset_gyro[0] = sum_gyro[0];
    _offset_gyro[1] = sum_gyro[1];
    _offset_gyro[2] = sum_gyro[2];
    _calibration = true;
    ESP_LOGI(TAG, "%s : Zeroing Completed. ",_name.c_str());

}

std::tuple<esp_err_t, std::array<float, 3>, std::array<float, 3>> ICM20948::read_with_offset()
{
    if (!_calibration){
        ESP_LOGW(TAG,"%s Calibration was not performed.",_name.c_str());
        return {ESP_FAIL,{},{}};
    }

    auto [ret,acc,gyro] = read_raw_data();
    acc[0]  = acc[0]  - _offset_acc[0];
    acc[1]  = acc[1]  - _offset_acc[1];
    acc[2]  = acc[2]  - _offset_acc[2];
    gyro[0] = gyro[0] - _offset_gyro[0];
    gyro[1] = gyro[1] - _offset_gyro[1];
    gyro[2] = gyro[2] - _offset_gyro[2];

    // ICM20948에서 부호를 반대로 설정해놨어요. 논리 대로 처리 간다~~~~  
    // qgc의 roll , pitch에 (-) 부호처리 일단함.
    // 오른손법칙에 어긋나는 부분 교정하여 Mahony에 입력한다 
    acc[1]  *=  -1.0f;
    gyro[0] *=  -1.0f;

    // 오른쪽으로 회전시 (-)부호로 값은 커진다. (즉 값이 작아진다는 것이다)
    gyro[2] *=  -1.0f;
    return {ret,acc,gyro};
}

std::tuple<esp_err_t, std::array<float, 3>, std::array<float, 3>> ICM20948::Managed_read_with_offset()
{
    static size_t active_index = 0;
    static size_t err_count = 0;        
    static size_t err_continue_count = 0;
    static float avr_acc[3] = {}, avr_gyro[3] = {};
    static bool is_fault_posted = false; // 이벤트 중복 발행 방지

    // [핵심] 두 센서 모두 임계치 초과 시 이벤트 발행
    if (err_continue_count > 6) {
        if (!is_fault_posted) {
            Service::fault_event_data_t data = { 
                .id = Service::FAULT_ID_IMU, 
                .is_recovered = false,
                .reason = ESP_ERR_TIMEOUT 
            };
            // Failsafe 모듈에게 "IMU 둘 다 먹통임"을 알림
            esp_event_post(Service::SYS_FAULT_EVENT_BASE, Service::SENSOR_EVENT_READ_FAILED, 
                           &data, sizeof(data), 0);
            is_fault_posted = true; 
            ESP_LOGE(TAG, "Both IMU sensors failed. Event posted.");
        }
        return {ESP_FAIL, {avr_acc[0], avr_acc[1], avr_acc[2]}, {avr_gyro[0], avr_gyro[1], avr_gyro[2]}}; 
    }

    // 센서 읽기 로직 (Main/Sub 스위칭)
    auto& target_instance = (active_index == 0) ? ICM20948::mainInstance : ICM20948::subInstance;
    auto [err, acc, gyro] = target_instance.read_with_offset();

    if (err == ESP_OK) {
        // 성공 시 데이터 업데이트 및 에러 카운트 초기화
        for(int i=0; i<3; i++) { avr_acc[i] = acc[i]; avr_gyro[i] = gyro[i]; }
        err_count = 0;
        err_continue_count = 0;

        // [핵심] 복구되었다면 복구 이벤트 발행
        if (is_fault_posted) {
            Service::fault_event_data_t data = { .id = Service::FAULT_ID_IMU, .is_recovered = true };
            esp_event_post(Service::SYS_FAULT_EVENT_BASE, Service::SENSOR_EVENT_READ_RECOVERED, &data, sizeof(data), 0);
            is_fault_posted = false;
        }
        return {ESP_OK, acc, gyro};
    } 
    else {
        // 실패 시 스위칭 로직
        err_count++;
        if (err_count > 3) {
            ESP_LOGW(TAG, "IMU %d failed, switching...", active_index);
            active_index = (active_index == 0) ? 1 : 0; // 스위칭
            err_count = 0;
            err_continue_count++;
        }
        return {err, {avr_acc[0], avr_acc[1], avr_acc[2]}, {avr_gyro[0], avr_gyro[1], avr_gyro[2]}};
    }
}


ICM20948::ICM20948()
{
    ESP_LOGI(TAG,"Initializing ICM20948 Sensor...");
}

esp_err_t ICM20948::enable_mag_bypass()
{
    esp_err_t err;
    icm20948_select_bank(0);
    // 1. I2C Master 모드 비활성화 (Bypass를 쓰기 위함)
    uint8_t user_ctrl_cmd[] = {B0_USER_CTRL, 0x00};
    err = i2c_master_transmit(_dev_handle, user_ctrl_cmd, 2, pdMS_TO_TICKS(10));
    if (err != ESP_OK){
        ESP_LOGI(TAG,"%s : I2C Master Mode Deactivation Fialed.",_name.c_str());
        return err;
    }

    // 2. Bypass 모드 활성화 (BYPASS_EN = 1)
    uint8_t bypass_cmd[] = {B0_INT_PIN_CFG, 0x02};
    err = i2c_master_transmit(_dev_handle, bypass_cmd, 2, pdMS_TO_TICKS(10));    
    if (err != ESP_OK) {
        ESP_LOGI(TAG,"%s : Bypass Mode Activation Failed.",_name.c_str());
        return err;
    }

    ESP_LOGI(TAG, "Geomagnetic Field (AK09916) Bypass Mode Activation Complete");
    return err;
}





} // namespace Sensor
