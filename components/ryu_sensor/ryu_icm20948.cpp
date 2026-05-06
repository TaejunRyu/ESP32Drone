#include "ryu_icm20948.h"

#include <tuple>
#include <freertos/FreeRTOS.h>
#include <esp_timer.h>
#include "ryu_i2c.h"
#include "ryu_sensor_event.h"
#include "ryu_businterface.h"

namespace Sensor{

//이 코드가 정상 동작하려면 SPIBus::read 함수가 구현된 파일에서 spi_device_interface_config_t 설정 시 address_bits = 8이 반드시 설정되어 있어야 합니다. 
//이 설정이 없다면 주소(B0_ACCEL_XOUT_H)가 전송되지 않아 엉뚱한 데이터를 읽게 됩니다.

ESP_EVENT_DEFINE_BASE(SYS_FAULT_EVENT_BASE);

ICM20948 ICM20948::mainInstance("ICM20948 Main",ICM20948::ADDR_VCC);
ICM20948 ICM20948::subInstance("ICM20948 Sub",ICM20948::ADDR_GND);


esp_err_t ICM20948::setup_i2c_interface(i2c_master_bus_handle_t bus_handle, uint16_t addr) {
    // 1. I2C 장치 등록 (열쇠 생성)
    i2c_master_dev_handle_t dev_h;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_h);
    if (ret != ESP_OK) return ret;

    // 기존에 할당된 버스가 있다면 정리 (선택 사항)
    if (_bus) delete _bus;
    
    // 2. I2CBus 인터페이스 객체 생성 및 주입 (도구 조립 및 전달)
    // static 혹은 멤버 변수로 관리하여 메모리 해제 방지
    _bus = new Interface::I2CBus(dev_h); 

    // 3. 센서 초기화 진행
    // this->initialize();
    return ESP_OK;
}


void ICM20948::icm20948_select_bank(uint8_t bank)
{
    //uint8_t cmd[] = {REG_BANK_SEL, (uint8_t)(bank << 4)};
    //i2c_master_transmit(_dev_handle, cmd, 2, pdMS_TO_TICKS(10));

    _bus->write(REG_BANK_SEL,(uint8_t)(bank << 4));
    _current_bank = bank;
}


esp_err_t ICM20948::initialize() {
    if (_initialized) {
        ESP_LOGI(TAG, "%s Already initialized.", _name.c_str());
        return ESP_OK;
    }

    // [중요] _bus가 외부에서 주입(set_bus)되었는지 먼저 확인합니다.
    if (_bus == nullptr) {
        ESP_LOGE(TAG, "%s : Bus interface not set!", _name.c_str());
        return ESP_FAIL;
    }

    esp_err_t err;

    // 0. 초기 뱅크 설정
    icm20948_select_bank(0);

    // 1. Soft Reset
    // 기존: i2c_master_transmit(_dev_handle, reset_cmd, 2, ...)
    // 변경: 인터페이스의 write(레지스터, 데이터) 사용
    err = _bus->write(B0_PWR_MGMT_1, 0x80); 
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s : Soft Reset setting failed", _name.c_str());
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // 2. Sleep 해제 및 Auto Clock 선택
    err = _bus->write(B0_PWR_MGMT_1, 0x01);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s : Sleep & Auto Clock setting failed", _name.c_str());
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // --- [Bank 2로 이동] ---
    icm20948_select_bank(2);
    vTaskDelay(pdMS_TO_TICKS(10));

    // --- [자이로스코프 설정] ---
    err = _bus->write(B2_GYRO_CONFIG_1, 0x1D);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s : Gyro DLPF setting failed", _name.c_str());
        return err;
    }

    // --- [가속도계 설정] ---
    err = _bus->write(B2_ACCEL_CONFIG, 0x1D);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s : Accel DLPF setting failed", _name.c_str());
        return err;
    }

    // --- [Bank 0로 복귀] ---
    icm20948_select_bank(0);
    vTaskDelay(pdMS_TO_TICKS(10));

    _initialized = true;
    _isAlive = true;
    ESP_LOGI(TAG, "%s Initialized successfully via Interface.", _name.c_str());
    return ESP_OK;
}


esp_err_t ICM20948::deinitialize() {
    // 1. 상태 체크
    if (!_initialized) {
        return ESP_OK;
    }

    // 2. 하드웨어 자원 해제 (중요!)
    // 만약 BusInterface 객체의 생명주기를 ICM20948이 관리한다면 여기서 delete 합니다.
    // 외부에서 관리한다면 단순히 포인터를 nullptr로 만듭니다.
    _bus = nullptr; 

    // 3. 상태 업데이트
    _initialized = false;
    _isAlive = false;

    ESP_LOGI(TAG, "%s Deinitialized successfully (Interface detached).", _name.c_str());
    return ESP_OK;
}



std::tuple<esp_err_t, std::array<float, 3>, std::array<float, 3>> ICM20948::read_raw_data() {
    if (_bus == nullptr) {
        return {ESP_FAIL, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    }

    esp_err_t ret;
    std::array<float, 3> acc, gyro;
    icm20948_select_bank(0);

    // [구분 처리] I2C vs SPI
    if (_bus->get_type() == Interface::BusType::I2C) {
        // I2C일 때는 기존처럼 Accel + Gyro (12바이트)만 읽음
        uint8_t d[12];
        ret = _bus->read(B0_ACCEL_XOUT_H, d, 12);
        if (ret != ESP_OK) return {ret, {}, {}};

        acc[0] = (int16_t)((d[0] << 8) | d[1]) / 4096.0f;
        acc[1] = (int16_t)((d[2] << 8) | d[3]) / 4096.0f;
        acc[2] = (int16_t)((d[4] << 8) | d[5]) / 4096.0f;

        gyro[0] = (int16_t)((d[6] << 8) | d[7]) / 32.8f;
        gyro[1] = (int16_t)((d[8] << 8) | d[9]) / 32.8f;
        gyro[2] = (int16_t)((d[10] << 8) | d[11]) / 32.8f;
    } 
    else {
        // SPI일 때는 Mag 포함 (21바이트) 읽음 (Accel 6 + Gyro 6 + Temp 2 + Mag 7)
        // 0x2D(Accel_X) ~ 0x41(Mag_Status2)까지 연속 읽기
        uint8_t d[21];
        ret = _bus->read(B0_ACCEL_XOUT_H, d, 21);
        if (ret != ESP_OK) return {ret, {}, {}};

        // Accel & Gyro 변환 (기존과 동일)
        acc[0] = (int16_t)((d[0] << 8) | d[1]) / 4096.0f;
        acc[1] = (int16_t)((d[2] << 8) | d[3]) / 4096.0f;
        acc[2] = (int16_t)((d[4] << 8) | d[5]) / 4096.0f;

        gyro[0] = (int16_t)((d[6] << 8) | d[7]) / 32.8f;
        gyro[1] = (int16_t)((d[8] << 8) | d[9]) / 32.8f;
        gyro[2] = (int16_t)((d[10] << 8) | d[11]) / 32.8f;

        // 지자계 데이터 변환 (d[14]부터 Mag 데이터 시작)
        // AK09916 데이터 포맷: Little-Endian 방식 주의
        // d[14]: ST1, d[15]: HXL, d[16]: HXH, ... d[20]: ST2
        _mag_data[0] = (int16_t)((d[16] << 8) | d[15]) * 0.15f; // uT 단위 변환
        _mag_data[1] = (int16_t)((d[18] << 8) | d[17]) * 0.15f;
        _mag_data[2] = (int16_t)((d[20] << 8) | d[19]) * 0.15f;
    }

    return {ret, acc, gyro};
}

void ICM20948::calibrate() {
    if (_calibration) {
        ESP_LOGW(TAG, "%s Already Calibration was performed.", _name.c_str());
        return;
    }

    // 인터페이스 연결 확인
    if (_bus == nullptr) {
        ESP_LOGE(TAG, "%s : Bus interface not set!", _name.c_str());
        return;
    }

    ESP_LOGI(TAG, "%s : Zeroing... Keep the aircraft level.", _name.c_str());
    
    float sum_acc[3] = {}, sum_gyro[3] = {};
    int valid_samples = 0;
    const int samples = 500;
    uint8_t buf[12];
    
    icm20948_select_bank(0);

    for (int i = 0; i < samples; i++) {
        uint64_t start_time = esp_timer_get_time();

        // [변경] i2c_master_transmit_receive 대신 인터페이스의 read 사용
        if (_bus->read(B0_ACCEL_XOUT_H, buf, 12) == ESP_OK) [[likely]] {
            sum_acc[0] += (int16_t)((buf[0] << 8) | buf[1]) / 4096.0f;
            sum_acc[1] += (int16_t)((buf[2] << 8) | buf[3]) / 4096.0f;
            sum_acc[2] += (int16_t)((buf[4] << 8) | buf[5]) / 4096.0f;
            sum_gyro[0] += (int16_t)((buf[6] << 8) | buf[7]) / 32.8f;
            sum_gyro[1] += (int16_t)((buf[8] << 8) | buf[9]) / 32.8f;
            sum_gyro[2] += (int16_t)((buf[10] << 8) | buf[11]) / 32.8f;
            valid_samples++;
        }

        // 주기 제어 (400Hz)
        uint64_t end_time = esp_timer_get_time();
        uint32_t elapsed = (uint32_t)(end_time - start_time);
        if (elapsed < 2500) {
            esp_rom_delay_us(2500 - elapsed);
        }
    }

    // ... (이하 평균값 계산 및 오프셋 저장 로직은 기존과 동일) ...
    if (valid_samples > 0) [[likely]] {
        _offset_acc[0] = sum_acc[0] / valid_samples;
        _offset_acc[1] = sum_acc[1] / valid_samples;
        _offset_acc[2] = (sum_acc[2] / valid_samples) - 1.0f;
        _offset_gyro[0] = sum_gyro[0] / valid_samples;
        _offset_gyro[1] = sum_gyro[1] / valid_samples;
        _offset_gyro[2] = sum_gyro[2] / valid_samples;
        _calibration = true;
        ESP_LOGI(TAG, "%s : Zeroing Completed.", _name.c_str());
    } else {
        ESP_LOGE(TAG, "%s : Calibration Failed (No valid samples).", _name.c_str());
    }
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
        for(int ii=0; ii<3; ii++) { avr_acc[ii] = acc[ii]; avr_gyro[ii] = gyro[ii]; }
        err_count = 0;
        err_continue_count = 0;

        // [핵심] 복구되었다면 복구 이벤트 발행
        if (is_fault_posted) {
            Service::fault_event_data_t data = { 
                .id = Service::FAULT_ID_IMU, 
                .is_recovered = true,
                .reason = ESP_ERR_TIMEOUT
            };
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


esp_err_t ICM20948::enable_mag_bypass() {
    if (_bus == nullptr) return ESP_FAIL;

    if (_bus->get_type() == Interface::BusType::I2C) {
        // --- [I2C 모드: 기존 Bypass 로직] ---
        icm20948_select_bank(0);
        _bus->write(B0_USER_CTRL, 0x00);    // I2C Master Off
        _bus->write(B0_INT_PIN_CFG, 0x02); // Bypass On
        ESP_LOGI(TAG, "%s Bypass Mode Enabled",_name.c_str());
    } 
    else if (_bus->get_type() == Interface::BusType::SPI) {
        // --- [SPI 모드: Internal I2C Master 로직] ---
        // 앞서 설명드린 Bank 3 슬레이브 설정 로직 실행
        icm20948_select_bank(0);
        _bus->write(B0_USER_CTRL, 0x20);    // I2C Master On
        
        icm20948_select_bank(3);
        _bus->write(B3_I2C_MST_CTRL, 0x07); // 400kHz
        _bus->write(B3_I2C_SLV0_ADDR, 0x0C | 0x80); // Mag Read
        _bus->write(B3_I2C_SLV0_REG, 0x11);
        _bus->write(B3_I2C_SLV0_CTRL, 0x89);
        
        icm20948_select_bank(0);
        ESP_LOGI(TAG, "SPI Internal I2C Master Enabled for Mag");
    }

    return ESP_OK;
}

} // namespace Sensor
