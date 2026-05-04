#include "ryu_ist8310.h"
#include <tuple>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ryu_i2c.h"

namespace Sensor
{

const char* IST8310::TAG = "IST8310";

IST8310::IST8310(){
    ESP_LOGI(TAG,"Initializing IST8310 Sensor...");
}

IST8310::~IST8310()
{
    deinitialize();
}


esp_err_t IST8310::initialize()
{
    this->_bus_handle = Driver::I2C::get_instance().get_bus_handle();

    i2c_device_config_t mag_cfg = {};
    mag_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    mag_cfg.device_address  = ADDR; // 스캔 결과 확인된 주소
    mag_cfg.scl_speed_hz    = Driver::I2C::I2C_SPEED;;

    esp_err_t err = i2c_master_bus_add_device(this->_bus_handle, &mag_cfg, &_dev_handle);
    if (err != ESP_OK){
        return err;
    }

    // 1. 소프트 리셋 (중요: 리셋 후 반드시 50ms 이상 대기)
    uint8_t reset_cmd[] = {CONTROL2, 0x01}; // CNTL2, Soft Reset
    err = i2c_master_transmit(_dev_handle, reset_cmd, 2, pdMS_TO_TICKS(50));
    if (err != ESP_OK){
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    //2. WHO_AM_I 확인 (정상 연결 체크)
    uint8_t who_reg = 0x00, who_val = 0;
    err = i2c_master_transmit_receive(_dev_handle, &who_reg, 1, &who_val, 1, pdMS_TO_TICKS(100));
    if (err != ESP_OK){
        return err;
    }
    if (who_val != 0x10) {
        ESP_LOGE(TAG, "연결 실패! ID: 0x%02X (기대값: 0x10)", who_val);
        return ESP_FAIL;
    }

    // 3. 센서 내부 동작 환경 설정 (이 루틴이 없으면 데이터 갱신 안됨)
    // AVGCNTL(0x41): 0x24 (X,Y,Z 모두 16회 평균으로 노이즈 제거)
    uint8_t avg_data[] = {AVGCNTL, 0x24};
    err = i2c_master_transmit(_dev_handle, avg_data, 2, pdMS_TO_TICKS(10));
    if (err != ESP_OK){
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    // PDCNTL(0x42): 0xC0 (Pulse Duration 권장값)
    uint8_t pd_data[] = {PDCNTL, 0xC0};
    err = i2c_master_transmit(_dev_handle, pd_data, 2, pdMS_TO_TICKS(10));
    if (err != ESP_OK){
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
 
    // CROSS  AXIS 1
    uint8_t mode_cmd_axis1[] = {CROSSAXIS1, 0x01};
    err = i2c_master_transmit(_dev_handle, mode_cmd_axis1, 2, pdMS_TO_TICKS(10));
    if (err != ESP_OK){
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    // CROSS  AXIS 2
    uint8_t mode_cmd_axis2[] = {CROSSAXIS2, 0x01};
    err = i2c_master_transmit(_dev_handle, mode_cmd_axis2, 2, pdMS_TO_TICKS(10));
    if (err != ESP_OK){
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    // 4. 모드 설정: 100Hz 연속 측정 모드 (CNTL1)
    // 0x08 = 100Hz Continuous Mode
    uint8_t mode_cmd_continuous[] = {CONTROL1, 0x0B};
    err = i2c_master_transmit(_dev_handle, mode_cmd_continuous, 2, pdMS_TO_TICKS(10));
    if (err != ESP_OK){
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    this->_initialized = true;
    ESP_LOGI(TAG, "Initialized successfully. (ID: 0x10, 100Hz mode)");
    return ESP_OK;
}


void IST8310::deinitialize()
{
    if (!this->_initialized){
        return;
    }
    if (this->_dev_handle){
        i2c_master_bus_rm_device(this->_dev_handle);
    }
    this->_initialized = false;
    ESP_LOGI(TAG, "deinitialzed.");
}


std::tuple<esp_err_t, std::array<float, 3>> IST8310::read_raw_data()
{
    uint8_t rx_buf[6] = {0};
    std::array<float, 3> raw_float = {0.0f, 0.0f, 0.0f};
    
    // 1. 6바이트 연속 읽기 (X_L, X_H, Y_L, Y_H, Z_L, Z_H)
    uint8_t reg_addr = DATA_X_L; // 0x03
    esp_err_t ret = i2c_master_transmit_receive(
        this->_dev_handle, 
        &reg_addr, 1, 
        rx_buf, 6, 
        pdMS_TO_TICKS(2) // 2ms 타임아웃
    );

    // 2. I2C 통신 실패 시 방어 코드
    if (ret != ESP_OK) {
        // 통신이 실패하면 드론이 추락하는 것을 막기 위해 '직전 정상 데이터'를 그대로 반환합니다.
        return {ret, last_valid_mag};
    }

    // 3. 바이트 결합 및 부호 있는 16비트 정수 변환 (Little Endian)
    // IST8310은 하위 바이트(Low)가 먼저 오고 상위 바이트(High)가 나중에 옵니다.
    int16_t x_raw = (int16_t)((rx_buf[1] << 8) | rx_buf[0]);
    int16_t y_raw = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
    int16_t z_raw = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);

    // 4. 감도 적용 (Gauss 또는 uT 단위 변환)
    // 헤더에 정의하신 SENSITIVITY(0.3f) 값을 곱해 물리량으로 바꿉니다.
    raw_float[0] = (float)x_raw * SENSITIVITY;
    raw_float[1] = (float)y_raw * SENSITIVITY;
    raw_float[2] = (float)z_raw * SENSITIVITY;

    // 5. 소프트웨어 IIR 로우패스 필터 적용 (드론 모터 노이즈 제거)
    if (last_valid_mag[0] == 0.0f && last_valid_mag[1] == 0.0f && last_valid_mag[2] == 0.0f) {
        // 최초 실행 시에는 필터링 없이 현재 값 저장
        last_valid_mag = raw_float;
    } else {
        // 이전 값과 현재 값의 가중치 평균을 구합니다.
        last_valid_mag[0] = last_valid_mag[0] + FILTER_ALPHA * (raw_float[0] - last_valid_mag[0]);
        last_valid_mag[1] = last_valid_mag[1] + FILTER_ALPHA * (raw_float[1] - last_valid_mag[1]);
        last_valid_mag[2] = last_valid_mag[2] + FILTER_ALPHA * (raw_float[2] - last_valid_mag[2]);
    }

    return {ESP_OK, last_valid_mag};
}

std::tuple<esp_err_t, std::array<float, 3>> IST8310::read_with_offset()
{
    auto [ret,mag_data] = this->read_raw_data();
    mag_data[0] = (mag_data[0]- MAG_OFFSET_X) * SCALE_X;
    mag_data[1] = (mag_data[1]- MAG_OFFSET_Y) * SCALE_Y;
    mag_data[2] = (mag_data[2]- MAG_OFFSET_Z) * SCALE_Z;

    float norm = sqrtf(mag_data[0] * mag_data[0] + mag_data[1] * mag_data[1] + mag_data[2] * mag_data[2]);
    if (norm > 0.0f) {
        mag_data[0] =mag_data[0]/norm;
        mag_data[1] =mag_data[1]/norm;
        mag_data[2] =mag_data[2]/norm;
    }
    
    // X를 (-)부호를 해야지 Mahony를 통과
    mag_data[0] *=  -1.0f;
    return {ret,mag_data};
}



void IST8310::calibrate_hard_iron()
{
    float mx_max = -99999.0f, mx_min = 99999.0f;
    float my_max = -99999.0f, my_min = 99999.0f;
    float mz_max = -99999.0f, mz_min = 99999.0f;

    ESP_LOGI(TAG, "지자계 보정 시작: 드론을 모든 방향(8자)으로 돌리세요 (약 60초)...");    
    // 약 10000 샘플링 (100Hz 기준 약 30초)
    for (int i = 0; i < 10000; i++) {
        float mx=0.0f, my=0.0f, mz=0.0f;    
        auto [ret,mag_raw]=this->read_raw_data();
        if (ret == ESP_OK) {
            mx = static_cast<float>(mag_raw[0]);
            my = static_cast<float>(mag_raw[1]);
            mz = static_cast<float>(mag_raw[2]);

            // 각 축의 최대/최소값 갱신
            if (mx > mx_max) mx_max = mx;
            if (mx < mx_min) mx_min = mx;
            if (my > my_max) my_max = my;
            if (my < my_min) my_min = my;
            if (mz > mz_max) mz_max = mz;
            if (mz < mz_min) mz_min = mz;
        }
        // 100Hz 주기를 맞추기 위한 딜레이
        vTaskDelay(pdMS_TO_TICKS(10));

        // 5초마다 진행 상황 출력
        if (i % 500 == 0) {
            printf("\rIST8310 : 보정 진행 중... (%d%%)", (i * 100) / 10000);
        }
    }
    // 최종 하드 아이언 오프셋(중심점) 계산
    float offset_x = (mx_max + mx_min) / 2.0f;
    float offset_y = (my_max + my_min) / 2.0f;
    float offset_z = (mz_max + mz_min) / 2.0f;

    // 센서 감도(Soft-Iron 성분 요약)를 위한 스케일 계산 (선택 사항)
    float avg_delta_x = (mx_max - mx_min) / 2.0f;
    float avg_delta_y = (my_max - my_min) / 2.0f;
    float avg_delta_z = (mz_max - mz_min) / 2.0f;
    float avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3.0f;

    float scale_x = avg_delta / avg_delta_x;
    float scale_y = avg_delta / avg_delta_y;
    float scale_z = avg_delta / avg_delta_z;

    printf("\nIST8310 : 보정 완료!\n\n");
    printf("inline constexpr float MAG_MAX_X      = %.2f;\n", mx_max);
    printf("inline constexpr float MAG_MAX_Y      = %.2f;\n", my_max);
    printf("inline constexpr float MAG_MAX_Z      = %.2f;\n", mz_max);   
    printf("inline constexpr float MAG_MIN_X      = %.2f;\n", mx_min);
    printf("inline constexpr float MAG_MIN_Y      = %.2f;\n", my_min);
    printf("inline constexpr float MAG_MIN_Z      = %.2f;\n\n", mz_min);
    printf("inline constexpr float SCALE_X        = %.2f;\n", scale_x);
    printf("inline constexpr float SCALE_Y        = %.2f;\n", scale_y);
    printf("inline constexpr float SCALE_Z        = %.2f;\n", scale_z);
    printf("inline constexpr float MAG_OFFSET_X   = %.2f;\n", offset_x);
    printf("inline constexpr float MAG_OFFSET_Y   = %.2f;\n", offset_y);
    printf("inline constexpr float MAG_OFFSET_Z   = %.2f;\n\n", offset_z);
}
}