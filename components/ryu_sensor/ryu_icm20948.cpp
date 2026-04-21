#include "ryu_icm20948.h"


namespace ICM20948{

inline constexpr uint8_t  REG_BANK_SEL     =  0x7F;
inline constexpr uint8_t  B0_WHO_AM_I      =  0x00; // 값: 0xEA
inline constexpr uint8_t  B0_PWR_MGMT_1    =  0x06;
inline constexpr uint8_t  B0_ACCEL_XOUT_H  =  0x2D; // 가속도 시작 (총 6바이트)
inline constexpr uint8_t  B0_GYRO_XOUT_H   =  0x33; // 자이로 시작 (총 6바이트)
// 뱅크 및 레지스터 추가 정의
inline constexpr uint8_t  B0_INT_PIN_CFG   =  0x0F;
inline constexpr uint8_t  B0_USER_CTRL     =  0x03; 
// 레지스터 정의 (Bank 2 기준)=
inline constexpr uint8_t  B2_GYRO_CONFIG_1 =  0x01;
inline constexpr uint8_t  B2_ACCEL_CONFIG  =  0x14;

// 뱅크 선택 함수
void icm20948_select_bank(i2c_master_dev_handle_t handle, uint8_t bank) {
    uint8_t cmd[] = {REG_BANK_SEL, (uint8_t)(bank << 4)};
    i2c_master_transmit(handle, cmd, 2, pdMS_TO_TICKS(10));
}

i2c_master_dev_handle_t initialize(i2c_master_bus_handle_t bus_handle,  uint16_t dev_address){

    i2c_master_dev_handle_t handle={};

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = dev_address;
    dev_cfg.scl_speed_hz = I2C_SPEED; // 400kHz

    // 버스에 장치 추가 및 핸들 획득
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &handle);
    if (ret != ESP_OK) {
        //ESP_LOGE("ICM20948", "장치 등록 실패: %s", esp_err_to_name(ret));
        return nullptr;
    }

    icm20948_select_bank(handle,0);
    
    // 1. Soft Reset
    uint8_t reset_cmd[] = {B0_PWR_MGMT_1, 0x80};
    i2c_master_transmit(handle, reset_cmd, 2, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(100)); // 리셋 후 대기 필수

    // 2. Sleep 해제 및 Auto Clock 선택 (내부 20MHz OSC 사용)
    uint8_t wake_cmd[] = {B0_PWR_MGMT_1, 0x01};
    i2c_master_transmit(handle, wake_cmd, 2, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // --- [1. Bank 2로 이동] ---
    icm20948_select_bank(handle,2);
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
    i2c_master_transmit(handle, gyro_cfg, 2, pdMS_TO_TICKS(100));

    // --- [3. 가속도계 설정: ±8g & DLPF 설정] ---
    /* 
       ACCEL_CONFIG (0x14) 설정값 계산:
       - Bit [5:3]: DLPF 대역폭 설정 (3 = 24.6Hz)
       - Bit [2:1]: Full Scale (2 = ±8g)
       - Bit [0]: DLPF 활성화 (1 = Enable)
       => 0b00011101 (0x1D)
    */
    uint8_t accel_cfg[] = {B2_ACCEL_CONFIG, 0x1D};
    i2c_master_transmit(handle, accel_cfg, 2, pdMS_TO_TICKS(100));

    // --- [4. 다시 Bank 0로 복귀 (데이터 읽기 준비)] ---
    icm20948_select_bank(handle,0);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    //ESP_LOGI("ICM20948", "범위 설정 완료: Accel ±8g, Gyro ±1000dps, DLPF ~24Hz");
    return handle;
}



/**
 * @brief ICM20948 내부의 AK09916(지자계)을 직접 접근하기 위한 Bypass 모드 활성화
 */
esp_err_t enable_mag_bypass(i2c_master_dev_handle_t handle) {
    
    icm20948_select_bank(handle,0);

    // 1. I2C Master 모드 비활성화 (Bypass를 쓰기 위함)
    uint8_t user_ctrl_cmd[] = {B0_USER_CTRL, 0x00};
    i2c_master_transmit(handle, user_ctrl_cmd, 2, pdMS_TO_TICKS(10));

    // 2. Bypass 모드 활성화 (BYPASS_EN = 1)
    uint8_t bypass_cmd[] = {B0_INT_PIN_CFG, 0x02};
    esp_err_t ret = i2c_master_transmit(handle, bypass_cmd, 2, pdMS_TO_TICKS(10));
    
    if (ret == ESP_OK) {
        ESP_LOGI("ICM20948", "지자계(AK09916) Bypass 모드 활성화 완료");
    }
    return ret;
}


extern "C" std::tuple < esp_err_t , std::array<float,3>, std::array<float,3>> 
        read_raw_data(i2c_master_dev_handle_t handle){
    if (handle == NULL) 
        return {ESP_FAIL, {0.0f, 0.0f, 0.0f},{0.0f,0.0f,0.0f}};
    uint8_t d[12]; // Accel(6) + Gyro(6)
    
    icm20948_select_bank(handle,0);
    esp_err_t ret = i2c_master_transmit_receive(handle, &B0_ACCEL_XOUT_H, 1, d, 12, pdMS_TO_TICKS(2));
    if(ret != ESP_OK){
        return{ret,{},{}};
    } 
    float raw_acc[3] = {},raw_gyro[3] ={};
    // Raw 데이터 결합 (Big-Endian)
    raw_acc[X]  = (int16_t)(d[0] << 8  | d[1])     / 4096.0f ;
    raw_acc[Y]  = (int16_t)(d[2] << 8  | d[3])     / 4096.0f ;
    raw_acc[Z]  = (int16_t)(d[4] << 8  | d[5])     / 4096.0f ;
    raw_gyro[X] = (int16_t)(d[6] << 8  | d[7])     / 32.8f   ;
    raw_gyro[Y] = (int16_t)(d[8] << 8  | d[9])     / 32.8f   ;
    raw_gyro[Z] = (int16_t)(d[10] << 8 | d[11])    / 32.8f   ;
    
    return {ret,{raw_acc[0],raw_acc[1],raw_acc[2]},{raw_gyro[0],raw_gyro[1],raw_gyro[2]}};
}

/**
 * @brief  
 * 
 * @param handle 
 * @return std::tuple<std::array<float,3>,std::array<float,3>> 
 */
std::tuple<std::array<float,3>,std::array<float,3>> calibrate( i2c_master_dev_handle_t handle) {

    float sum_acc[3] ={},sum_gyro[3]={};
     int valid_samples = 0; // 실제로 성공한 샘플 수만 카운트
    const int samples = 1000; // 1000번 샘플링 (약 1~2초 소요)
    uint8_t buf[12];
    uint8_t reg = B0_ACCEL_XOUT_H;
    
    icm20948_select_bank(handle,0);

    ESP_LOGI("ICM20948", "센서 영점 조절 시작... 기체를 수평으로 유지하세요.");
    for (int i = 0; i < samples; i++) {
        uint64_t start_time = esp_timer_get_time(); // 시작 시간 기록
        if (i2c_master_transmit_receive(handle, &reg, 1, buf, 12, pdMS_TO_TICKS(2)) == ESP_OK) [[likely]]{    
            sum_acc[X] += (int16_t)((buf[0] << 8) | buf[1])   / 4096.0f;
            sum_acc[Y] += (int16_t)((buf[2] << 8) | buf[3])   / 4096.0f;
            sum_acc[Z] += (int16_t)((buf[4] << 8) | buf[5])   / 4096.0f;
            
            sum_gyro[X] += (int16_t)((buf[6] << 8) | buf[7])   / 32.8f;  
            sum_gyro[Y] += (int16_t)((buf[8] << 8) | buf[9])   / 32.8f;
            sum_gyro[Z] += (int16_t)((buf[10] << 8) | buf[11]) / 32.8f;
            valid_samples++; 
        }  
        // 400Hz(2500us) 주기를 정확히 맞추기 위한 대기
        uint64_t end_time = esp_timer_get_time();
        uint32_t elapsed = (uint32_t)(end_time - start_time);
        if (elapsed < 2500) {
            esp_rom_delay_us(2500 - elapsed); // 남은 시간만큼 마이크로초 단위 대기
        }    
        //printf("\r         ICM20948: 센서 영점 조절 작업중. 진행율(%d/%d), valid samples(%d) ", i+1,samples,valid_samples);
        //ESP_LOGI("ICM20948", "센서 영점 조절 작업중.%d/%d",i,samples);
        //vTaskDelay(pdMS_TO_TICKS(2)); // 200Hz 이상  빠르게 샘플링
    }
    //printf("\n");
    if(valid_samples > 0 )[[likely]]{
        // 평균값 계산 (Offset 저장)
        sum_acc[X]  = sum_acc[X] / valid_samples;
        sum_acc[Y]  = sum_acc[Y] / valid_samples;
        sum_acc[Z]  = (sum_acc[Z] / valid_samples) -1.0f;  // 가속도는 중력 가속도(1g)를 제외하고 0이 되어야 함m

        sum_gyro[X] = sum_gyro[X] / valid_samples;
        sum_gyro[Y] = sum_gyro[Y] / valid_samples;
        sum_gyro[Z] = sum_gyro[Z] / valid_samples;
    }else{
        sum_acc[X] =0.0f;sum_acc[Y] =0.0f;sum_acc[Z] =0.0f; 
        sum_gyro[X] = 0.0f,sum_gyro[Y] = 0.0f,sum_gyro[Z] = 0.0f;
    }
    
    return {{sum_acc[X],sum_acc[Y],sum_acc[Z]},{sum_gyro[X],sum_gyro[Y],sum_gyro[Z]}};
}


}





// #include "driver/gpio.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// #define ICM_INT_GPIO_PIN   GPIO_NUM_4  // INT 핀이 연결된 ESP32 GPIO 번호
// static TaskHandle_t imu_task_handle = NULL;

// // 2.1 인터럽트 서비스 루틴 (ISR)
// static void IRAM_ATTR icm20948_isr_handler(void* arg) {
//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
//     // 데이터를 처리할 Task에 Notification을 보냅니다.
//     vTaskNotifyGiveFromISR(imu_task_handle, &xHigherPriorityTaskWoken);
    
//     if (xHigherPriorityTaskWoken) {
//         portYIELD_FROM_ISR();
//     }
// }

// // 2.2 GPIO 설정 및 ISR 등록 함수
// void setup_gpio_interrupt(void) {
//     gpio_config_t io_conf = {
//         .intr_type = GPIO_INTR_POSEDGE,       // ICM-20948 설정에 따라 NEGEDGE 또는 POSEDGE 선택
//         .mode = GPIO_MODE_INPUT,
//         .pin_bit_mask = (1ULL << ICM_INT_GPIO_PIN),
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .pull_up_en = GPIO_PULLUP_ENABLE      // Open-drain 설정 시 Pull-up 필요
//     };
//     gpio_config(&io_conf);

//     // GPIO 인터럽트 서비스 설치 (보통 app_main에서 한 번만 호출)
//     gpio_install_isr_service(0);
    
//     // 특정 GPIO 핀에 ISR 핸들러 부착
//     gpio_isr_handler_add(ICM_INT_GPIO_PIN, icm20948_isr_handler, NULL);
// }


// #include "driver/i2c_master.h"

// #define ICM20948_I2C_ADDR       0x68    // AD0 핀이 GND일 때 0x68, VCC일 때 0x69
// #define ICM20948_ACCEL_XOUT_H   0x2D    // Bank 0에 위치한 가속도 데이터 시작 주소

// i2c_master_dev_handle_t icm_dev_handle; // 이미 초기화된 I2C 디바이스 핸들

// // 3.1 인터럽트 트리거 시 데이터를 읽어오는 Task
// void imu_read_task(void *pvParameters) {
//     uint8_t data[12]; // 가속도(6B) + 자이로(6B)
    
//     while (1) {
//         // ISR로부터 신호가 올 때까지 대기 (무한 대기 portMAX_DELAY)
//         ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
//         // I2C 읽기 작업 수행
//         // Bank 0가 선택되어 있는지 확인 후 읽어야 안전합니다.
//         esp_err_t err = i2c_master_transmit_receive(
//             icm_dev_handle, 
//             (uint8_t[]){ICM20948_ACCEL_XOUT_H}, 1, 
//             data, sizeof(data), 
//             pdMS_TO_TICKS(100)
//         );

//         if (err == ESP_OK) {
//             // 16비트 정수로 조합 (Big Endian)
//             int16_t accel_x = (data[0] << 8) | data[1];
//             int16_t accel_y = (data[2] << 8) | data[3];
//             int16_t accel_z = (data[4] << 8) | data[5];
            
//             int16_t gyro_x  = (data[6] << 8) | data[7];
//             int16_t gyro_y  = (data[8] << 8) | data[9];
//             int16_t gyro_z  = (data[10] << 8) | data[11];

//             // 데이터 활용 (예: 출력)
//             // printf("Accel: %d, %d, %d\n", accel_x, accel_y, accel_z);
//         }
//     }
// }

// // 3.2 메인 및 초기화 흐름 예시
// void app_main(void) {
//     // 1. I2C 버스 및 센서 디바이스 등록 (코드 생략)
//     // 2. ICM-20948 내부 레지스터 설정 (Bank 선택, Data Ready INT 활성화 등)
    
//     // 3. Task 생성
//     xTaskCreate(imu_read_task, "imu_read_task", 4096, NULL, 10, &imu_task_handle);
    
//     // 4. GPIO 인터럽트 설정 및 가동
//     setup_gpio_interrupt();
// }
