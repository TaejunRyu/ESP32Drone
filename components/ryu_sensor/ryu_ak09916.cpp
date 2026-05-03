#include "ryu_ak09916.h"

#include <array>
#include <cmath>
#include <algorithm>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ryu_i2c.h"


namespace Sensor
{

const char* AK09916::TAG = "AK09916";

AK09916::AK09916():_initialized(false)
{
}

AK09916::~AK09916()
{
}

void AK09916::deinitialize()
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

i2c_master_dev_handle_t AK09916::initialize()
{
    if(_initialized){
        return _dev_handle;
    } 
    _bus_handle = Driver::I2C::get_instance().get_bus_handle();

    // AK09916 디바이스 추가 (I2C 버스에 직접 연결된 것처럼 동작)
    i2c_device_config_t mag_cfg = {};
    mag_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    mag_cfg.device_address  = ADDR;
    mag_cfg.scl_speed_hz    = Driver::I2C::I2C_SPEED;;

    if (i2c_master_bus_add_device(_bus_handle, &mag_cfg, &_dev_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Bus 추가 실패");
        return nullptr;
    }

    // 3. WHO_AM_I 확인 (AK09916의 ID는 0x09)
    uint8_t who_reg = WHO_AM_I;
    uint8_t who_val = 0;
    i2c_master_transmit_receive(_dev_handle, &who_reg, 1, &who_val, 1, pdMS_TO_TICKS(100));
    
    if (who_val != 0x09) {
        ESP_LOGE(TAG, "연결 실패! ID: 0x%02X (기대값: 0x09)", who_val);
        return nullptr;
    }
    //4. AK09916 소프트 리셋 및 모드 설정
    uint8_t reset_cmd[] = {CNTL3, 0x01};
    i2c_master_transmit(_dev_handle, reset_cmd, 2, pdMS_TO_TICKS(50));
    vTaskDelay(pdMS_TO_TICKS(50));

    // CNTL2: 0x08 (Continuous measurement mode 4 - 100Hz)
    uint8_t mode_cmd[] = {CNTL2, 0x08}; 
    i2c_master_transmit(_dev_handle, mode_cmd, 2, pdMS_TO_TICKS(100));
    
    _initialized = true;
    ESP_LOGI(TAG, "초기화 성공 (ID: 0x09)");
    return _dev_handle;
}

std::tuple<esp_err_t, std::array<float, 3>> AK09916::read_data()
{
    if (!_initialized){
        //
        return {ESP_FAIL,{}};
    }
    // 1. HXL(0x11)부터 ST2(0x18)까지 총 8바이트를 읽음
    // 이렇게 해야 ST2가 읽히면서 다음 샘플링이 시작됩니다.
    uint8_t buf[8];     // X(2) + Y(2) + Z(2) + TMPS(1) + ST2(1)

    esp_err_t ret = i2c_master_transmit_receive(_dev_handle, &HXL, 1, buf, 8, pdMS_TO_TICKS(2));
 
    if (ret == ESP_OK) {
        // 2. ST2 레지스터(buf[7]) 확인 (HOFL 비트 체크용, 필수는 아님)
        // 3. Little Endian 결합
        int16_t raw_x = (int16_t)((buf[1] << 8) | buf[0]) ; 
        int16_t raw_y = (int16_t)((buf[3] << 8) | buf[2]) ; 
        int16_t raw_z = (int16_t)((buf[5] << 8) | buf[4]) ; 

        // 3축의 절대값이 4912를 넘기면 자기 센서 오버플로이다(메뉴얼 기재됨.)
        //float fault_data = abs(raw_x) + abs(raw_y) + abs(raw_z);
        //if (fault_data > 4912) return{ESP_FAIL,{}};

        // 4. 일단 값이 변하는지 확인하기 위해 raw 값을 그대로 리턴
        return {ESP_OK, {static_cast<float>(raw_x), static_cast<float>(raw_y), static_cast<float>(raw_z)}};
    }
    return {ESP_FAIL, {}};
}

std::tuple<esp_err_t, std::array<float, 3>> AK09916::read_with_offset()
{
    auto [ret,mag_data] = read_data();
    mag_data[0] = (mag_data[0]- MAG_OFFSET_X) * SCALE_X;
    mag_data[1] = (mag_data[1]- MAG_OFFSET_Y) * SCALE_Y;
    mag_data[2] = (mag_data[2]- MAG_OFFSET_Z) * SCALE_Z;

    float norm = sqrtf(mag_data[0] * mag_data[0] + mag_data[1] * mag_data[1] + mag_data[2] * mag_data[2]);
    if (norm > 0.0f) {
        mag_data[0] =mag_data[0]/norm;
        mag_data[1] =mag_data[1]/norm;
        mag_data[2] =mag_data[2]/norm;
    }

    //
    float cal_x = mag_data[0] * -1.0f;
    float cal_y = mag_data[1] * -1.0f;
    float cal_z = mag_data[2];// * -1.0f;

    return {ret,{cal_x,cal_y,cal_z}};
}

std::tuple<esp_err_t, uint8_t> AK09916::ready_data()
{
    uint8_t data = 0x00;
    esp_err_t ret_st  = i2c_master_transmit_receive(_dev_handle, &STAT1, 1, &data, 1,  pdMS_TO_TICKS(1));
    return {ret_st,data};
}

void AK09916::calibrate_hard_iron()
{
    float mx=0.0f, my=0.0f, mz=0.0f;
    
    float mx_max = -99999.0f, mx_min = 99999.0f;
    float my_max = -99999.0f, my_min = 99999.0f;
    float mz_max = -99999.0f, mz_min = 99999.0f;

    ESP_LOGI(TAG, "지자계 보정 시작: 드론을 모든 방향(8자)으로 돌리세요 (약 30초)...");
    
    uint8_t buf[8];     // X(2) + Y(2) + Z(2) + TMPS(1) + ST2(1)

    // 약 3000번 샘플링 (100Hz 기준 약 30초)
    for (int i = 0; i < 5000; i++) {

        esp_err_t ret = i2c_master_transmit_receive(_dev_handle, &HXL, 1, buf, 8, pdMS_TO_TICKS(2));

        if (ret == ESP_OK) {
            // 2. ST2 레지스터(buf[7]) 확인 (HOFL 비트 체크용, 필수는 아님)
            // 3. Little Endian 결합
            int16_t raw_x = (int16_t)((buf[1] << 8) | buf[0]) ; 
            int16_t raw_y = (int16_t)((buf[3] << 8) | buf[2]) ; 
            int16_t raw_z = (int16_t)((buf[5] << 8) | buf[4]) ; 

            mx = (float)raw_x ;
            my = (float)raw_y ;
            mz = (float)raw_z ;

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
            printf("\rAK09916 : 보정 진행 중... (%d%%)", (i * 100) / 5000);
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

    printf("\nAK09916 : 보정 완료!\n\n");
    
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

}// namespace Sensor