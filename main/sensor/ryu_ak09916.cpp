#include "sensor/ryu_ak09916.h"


namespace AK09916
{
inline constexpr uint8_t HXL        =    0x11;// 데이터 시작 (X-axis Low)
inline constexpr uint8_t CNTL2      =    0x31;// 모드 설정 (100Hz 등)
inline constexpr uint8_t CNTL3      =    0x32;// 소프트 리셋
inline constexpr uint8_t WHO_AM_I   =    0x01;// ID 확인용 (값: 0x09)

// #define MAG_OFFSET_X -53.00
// #define MAG_OFFSET_Y 67.50
// #define MAG_OFFSET_Z 222.50
// Scale Factor (Soft-Iron): X:1.00, Y:0.99, Z:1.01


i2c_master_dev_handle_t initialize(i2c_master_bus_handle_t bus_handle) {

    i2c_master_dev_handle_t handle = {};

    // AK09916 디바이스 추가 (I2C 버스에 직접 연결된 것처럼 동작)
    i2c_device_config_t mag_cfg = {};
    mag_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    mag_cfg.device_address  = ADDR;
    mag_cfg.scl_speed_hz    = 400000;
    //mag_cfg.flags.disable_ack_check =true;


    if (i2c_master_bus_add_device(bus_handle, &mag_cfg, &handle) != ESP_OK) {
        ESP_LOGE("AK09916", "Bus 추가 실패");
        return nullptr;
    }

    // 3. WHO_AM_I 확인 (AK09916의 ID는 0x09)
    // uint8_t who_reg = WHO_AM_I;
    // uint8_t who_val = 0;
    // i2c_master_transmit_receive(handle, &who_reg, 1, &who_val, 1, pdMS_TO_TICKS(100));
    
    // if (who_val != 0x09) {
    //     ESP_LOGE("AK09916", "연결 실패! ID: 0x%02X (기대값: 0x09)", who_val);
    //     return nullptr;
    // }
    // 4. AK09916 소프트 리셋 및 모드 설정
    uint8_t reset_cmd[] = {CNTL3, 0x01};
    i2c_master_transmit(handle, reset_cmd, 2, pdMS_TO_TICKS(50));
    vTaskDelay(pdMS_TO_TICKS(50));

    // CNTL2: 0x08 (Continuous measurement mode 4 - 100Hz)
    uint8_t mode_cmd[] = {CNTL2, 0x08}; 
    i2c_master_transmit(handle, mode_cmd, 2, pdMS_TO_TICKS(100));
    
    ESP_LOGI("AK09916", "초기화 성공 (ID: 0x09)");
    return handle;
}
 

/**
 * @brief 
 *  
 * 
 * @param handle 
 * @return std::tuple<esp_err_t, std::array<float, 3>> 
 */
std::tuple<esp_err_t, std::array<float, 3>> read_data(i2c_master_dev_handle_t handle) {
    if (handle == NULL) return {ESP_FAIL, {}};

    // 1. HXL(0x11)부터 ST2(0x18)까지 총 8바이트를 읽음
    // 이렇게 해야 ST2가 읽히면서 다음 샘플링이 시작됩니다.
    uint8_t buf[8];     // X(2) + Y(2) + Z(2) + TMPS(1) + ST2(1)

    esp_err_t ret = i2c_master_transmit_receive(handle, &HXL, 1, buf, 8, pdMS_TO_TICKS(2));
 
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


/**
 * @brief 
 *  지자계 하드 아이언 보정 함수 
 *  사용자에게 8자 운동을 시키면서 최대/최소값을 수집하여 오프셋 계산.
 * @param handle 
 */
void calibrate_hard_iron(i2c_master_dev_handle_t handle) {
    float mx=0.0f, my=0.0f, mz=0.0f;
    
    float mx_max = -99999.0f, mx_min = 99999.0f;
    float my_max = -99999.0f, my_min = 99999.0f;
    float mz_max = -99999.0f, mz_min = 99999.0f;

    ESP_LOGI("AK09916", "지자계 보정 시작: 드론을 모든 방향(8자)으로 돌리세요 (약 30초)...");
    
    uint8_t buf[8];     // X(2) + Y(2) + Z(2) + TMPS(1) + ST2(1)

    // 약 3000번 샘플링 (100Hz 기준 약 30초)
    for (int i = 0; i < 5000; i++) {

        esp_err_t ret = i2c_master_transmit_receive(handle, &HXL, 1, buf, 8, pdMS_TO_TICKS(2));

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
}