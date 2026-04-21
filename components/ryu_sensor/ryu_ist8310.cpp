#include "ryu_ist8310.h"
#include <tuple>
namespace IST8310
{
    // cross axis 적용시 
// #define MAG_IST8310_MAX_X 69.51
// #define MAG_IST8310_MAX_Y 54.12
// #define MAG_IST8310_MAX_Z 28.90

// #define MAG_IST8310_MIN_X -67.95
// #define MAG_IST8310_MIN_Y -38.49
// #define MAG_IST8310_MIN_Z -72.33

// #define MAG_IST8310_OFFSET_X 0.78
// #define MAG_IST8310_OFFSET_Y 7.82
// #define MAG_IST8310_OFFSET_Z -21.71

// #define SCALE_IST8310_X 0.80
// #define SCALE_IST8310_Y 1.19
// #define SCALE_IST8310_Z 1.09

//float c12,c13,c21,c23,c31,c32;

// 이전 정상 데이터를 저장할 정적(static) 변수 (I2C 에러 대피용)
static std::array<float, 3> last_valid_mag = {0.0f, 0.0f, 0.0f};

// IIR 필터 계수 (0.0f ~ 1.0f 사이, 값이 작을수록 부드럽지만 반응이 느려짐)
// 20Hz 주기에서 0.3f~0.5f 정도가 드론 나침반용으로 가장 적당합니다.
inline constexpr float FILTER_ALPHA = 0.4f; 


i2c_master_dev_handle_t initialize(i2c_master_bus_handle_t bus_handle) {
    i2c_master_dev_handle_t handle = {};
    i2c_device_config_t mag_cfg = {};
    mag_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    mag_cfg.device_address  = ADDR; // 스캔 결과 확인된 주소
    mag_cfg.scl_speed_hz    = 400000;

    if (i2c_master_bus_add_device(bus_handle, &mag_cfg, &handle) != ESP_OK) return NULL;

    // 1. 소프트 리셋 (중요: 리셋 후 반드시 50ms 이상 대기)
    uint8_t reset_cmd[] = {CONTROL2, 0x01}; // CNTL2, Soft Reset
    i2c_master_transmit(handle, reset_cmd, 2, pdMS_TO_TICKS(50));
    vTaskDelay(pdMS_TO_TICKS(100));

    //2. WHO_AM_I 확인 (정상 연결 체크)
    uint8_t who_reg = 0x00, who_val = 0;
    i2c_master_transmit_receive(handle, &who_reg, 1, &who_val, 1, pdMS_TO_TICKS(100));
    if (who_val != 0x10) {
        ESP_LOGE("IST8310", "연결 실패! ID: 0x%02X (기대값: 0x10)", who_val);
        return nullptr;
    }

    // 3. 센서 내부 동작 환경 설정 (이 루틴이 없으면 데이터 갱신 안됨)
    // AVGCNTL(0x41): 0x24 (X,Y,Z 모두 16회 평균으로 노이즈 제거)
    uint8_t avg_data[] = {AVGCNTL, 0x24};
    i2c_master_transmit(handle, avg_data, 2, pdMS_TO_TICKS(10));
    vTaskDelay(pdMS_TO_TICKS(50));

    // PDCNTL(0x42): 0xC0 (Pulse Duration 권장값)
    uint8_t pd_data[] = {PDCNTL, 0xC0};
    i2c_master_transmit(handle, pd_data, 2, pdMS_TO_TICKS(10));
    vTaskDelay(pdMS_TO_TICKS(50));
 
    // CROSS  AXIS 1
    uint8_t mode_cmd_axis1[] = {CROSSAXIS1, 0x01};
    i2c_master_transmit(handle, mode_cmd_axis1, 2, pdMS_TO_TICKS(10));
    vTaskDelay(pdMS_TO_TICKS(50));

    // CROSS  AXIS 2
    uint8_t mode_cmd_axis2[] = {CROSSAXIS2, 0x01};
    i2c_master_transmit(handle, mode_cmd_axis2, 2, pdMS_TO_TICKS(10));
    vTaskDelay(pdMS_TO_TICKS(50));

    // 4. 모드 설정: 100Hz 연속 측정 모드 (CNTL1)
    // 0x08 = 100Hz Continuous Mode
    uint8_t mode_cmd_continuous[] = {CONTROL1, 0x0B};
    i2c_master_transmit(handle, mode_cmd_continuous, 2, pdMS_TO_TICKS(10));
    vTaskDelay(pdMS_TO_TICKS(50));

    // I2C Master 읽기 함수 예시 (HAL 또는 SDK 함수 사용)
    // uint8_t otp_data[18];
    // uint8_t reg = 0x40;
    // i2c_master_transmit_receive(handle,&reg,1,otp_data,18, pdMS_TO_TICKS(10));

    // // 계수 추출 (데이터시트 Table 10 기준 매핑)
    // c12 = (float)otp_data[2]  * 0.001953125f; // Cross-axis X to Y
    // c13 = (float)otp_data[4]  * 0.001953125f; // Cross-axis X to Z
    // c21 = (float)otp_data[8]  * 0.001953125f; // Cross-axis Y to X
    // c23 = (float)otp_data[10] * 0.001953125f; // Cross-axis Y to Z
    // c31 = (float)otp_data[14] * 0.001953125f; // Cross-axis Z to X
    // c32 = (float)otp_data[16] * 0.001953125f; // Cross-axis Z to Y
    //printf("c12:%f|c13:%f|c21:%f|c23:%f|c31:%f|c32:%f",c12,c13,c21,c23,c31,c32);
    ESP_LOGI("IST8310", "초기화 성공 (ID: 0x10, 100Hz 모드)");
    return handle;
}

// std::tuple<esp_err_t, std::array<float, 3>> read_raw_data(i2c_master_dev_handle_t handle) {
//     if (handle == nullptr) return {ESP_FAIL, {}};
//     uint8_t buf[7] = {0,};
//     // 7바이트를 한 번의 트랜잭션으로 읽음 
//     esp_err_t ret = i2c_master_transmit_receive(handle, &DATA_X_L, 1, buf, 7, pdMS_TO_TICKS(2));

//     //[Raw Data] → [Cross-axis 보정(OTP)] → [Hard-iron 오프셋 차감] → [정규화(Normalize)] → [Mahony 필터 입력]
    
//     if (ret == ESP_OK) {
//         // 데이터가 변하는지 확인하기 위해 buf[0] 출력 (확인용)
//         // printf("buf[0]: %d\n", buf[0]); 

//         int16_t raw_x = static_cast<int16_t>((buf[1] << 8) | buf[0]) ;
//         int16_t raw_y = static_cast<int16_t>((buf[3] << 8) | buf[2]) ;
//         int16_t raw_z = static_cast<int16_t>((buf[5] << 8) | buf[4]) ;

//         // 2. Cross-axis 보정 적용 (중요!)
//         // float adj_x = (float)raw_x          + (c12 * (float)raw_y) + (c13 * (float)raw_z);
//         // float adj_y = (c21 * (float)raw_x)  + (float)raw_y         + (c23 * (float)raw_z);
//         // float adj_z = (c31 * (float)raw_x)  + (c32 * (float)raw_y) + (float)raw_z;

//         // float final_x = adj_x /* * IST8310_SENSITIVITY */;
//         // float final_y = adj_y /* * IST8310_SENSITIVITY */;
//         // float final_z = adj_z /* * IST8310_SENSITIVITY */;
 
//         float final_x = static_cast<float>(raw_x)  * SENSITIVITY ;
//         float final_y = static_cast<float>(raw_y)  * SENSITIVITY ;
//         float final_z = static_cast<float>(raw_z)  * SENSITIVITY ;
 
//         return {ESP_OK, {final_x , final_y, final_z}};
//     }
//     return {ESP_FAIL, {}};
// }
 
/**
 * @brief IST8310으로부터 필터링된 Raw 데이터를 연속으로 읽어옵니다.
 */
std::tuple<esp_err_t, std::array<float, 3>> read_raw_data(i2c_master_dev_handle_t handle) {
    uint8_t reg_addr = DATA_X_L; // 0x03
    uint8_t rx_buf[6] = {0};
    std::array<float, 3> raw_float = {0.0f, 0.0f, 0.0f};

    // 1. 6바이트 연속 읽기 (X_L, X_H, Y_L, Y_H, Z_L, Z_H)
    esp_err_t ret = i2c_master_transmit_receive(
        handle, 
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


/**
 * @brief 
 *  지자계 하드 아이언 보정 함수 
 *  사용자에게 8자 운동을 시키면서 최대/최소값을 수집하여 오프셋 계산.
 * @param handle 
 */
void calibrate_hard_iron(i2c_master_dev_handle_t handle) {
    //uint8_t buf[7] = {0,};
        
    float mx_max = -99999.0f, mx_min = 99999.0f;
    float my_max = -99999.0f, my_min = 99999.0f;
    float mz_max = -99999.0f, mz_min = 99999.0f;

    ESP_LOGI("IST8310", "지자계 보정 시작: 드론을 모든 방향(8자)으로 돌리세요 (약 60초)...");    
    // 약 10000 샘플링 (100Hz 기준 약 30초)
    for (int i = 0; i < 10000; i++) {
        float mx=0.0f, my=0.0f, mz=0.0f;    
        auto [ret,mag_raw]=read_raw_data(handle);
        //esp_err_t ret = i2c_master_transmit_receive(handle, &IST8310_DATA_X_L, 1, buf, 7, pdMS_TO_TICKS(20));
        if (ret == ESP_OK) {
            // 2. ST2 레지스터(buf[7]) 확인 (HOFL 비트 체크용, 필수는 아님)
            // 3. Little Endian 결합
            // int16_t raw_x = static_cast<int16_t>((buf[1] << 8) | buf[0]) ; 
            // int16_t raw_y = static_cast<int16_t>((buf[3] << 8) | buf[2]) ; 
            // int16_t raw_z = static_cast<int16_t>((buf[5] << 8) | buf[4]) ; 

            mx = static_cast<float>(mag_raw[X]);
            my = static_cast<float>(mag_raw[Y]);
            mz = static_cast<float>(mag_raw[Z]);

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


/* 메인 루프에서의 사용 예시 */
/*
void app_main_loop() {
    uint8_t stat_reg = IST8310_STAT1;
    uint8_t stat_data = 0;

    // 1. 데이터 준비 확인 (DRDY 비트 체크)
    esp_err_t ret_st = i2c_master_transmit_receive(ist_handle, &stat_reg, 1, &stat_data, 1, pdMS_TO_TICKS(10));

    if (ret_st == ESP_OK && (stat_data & 0x01)) { 
        auto [ret, mag_val] = read_ist8310(ist_handle);
        if (ret == ESP_OK) {
            // mag_val[0], [1], [2] 사용 (raw 또는 보정 데이터)
            // ESP_LOGI("MAG", "X:%.2f Y:%.2f Z:%.2f", mag_val[0], mag_val[1], mag_val[2]);
        }
    }
}
*/
}