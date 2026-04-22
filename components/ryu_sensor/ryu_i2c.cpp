#include "ryu_i2c.h"
#include "ryu_bmp388.h"
namespace I2C{
/**
 * @brief I2C 마스터 버스를 초기화하고 핸들을 반환합니다.
 * 
 * @param port I2C 포트 번호
 * @param port_sda SDA GPIO 번호
 * @param port_scl SCL GPIO 번호
 * @return i2c_master_bus_handle_t 생성된 핸들 (실패 시 NULL)
 */
i2c_master_bus_handle_t initialize(i2c_port_num_t port, gpio_num_t port_sda, gpio_num_t port_scl)
{
    i2c_master_bus_handle_t bus_handle = NULL; // 핸들 변수 선언 및 초기화

    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = port;
    bus_cfg.sda_io_num = port_sda;
    bus_cfg.scl_io_num = port_scl;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.intr_priority = 1;
    bus_cfg.flags.enable_internal_pullup = true;;
    bus_cfg.flags.allow_pd = false;

    // 내부에서 생성한 bus_handle의 주소를 전달
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &bus_handle);

    if (ret != ESP_OK)
    {
        return nullptr; // 실패 시 NULL 반환
    }
   // ESP_LOGI("I2C", "✓ I2C 버스 초기화 완료");
    return bus_handle; // 생성된 핸들 반환
}


void scan_bus(i2c_master_bus_handle_t handle) {
    printf("\n     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");    
    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++) {
            uint8_t addr = i + j;
            
            // 주소 범위 체크 (I2C 표준: 0x03 ~ 0x77 사이만 유효)
            if (addr < 0x03 || addr > 0x77) {
                printf("   ");
                continue;
            }

            // i2c_master_probe: 장치가 응답(ACK)하는지 확인하는 전용 함수
            // 타임아웃은 10ms 정도면 충분합니다.
            esp_err_t ret = i2c_master_probe(handle, addr, pdMS_TO_TICKS(10));

            if (ret == ESP_OK) {
                printf("%02x ", addr);
            } else {
                printf("-- ");
            }
        }
        printf("\n");
    }
    printf("\n");
    vTaskDelay(pdMS_TO_TICKS(5000)); // 스캔 후 잠시 대기
}

/**
 * @brief   1)기존에 사용하던 i2c handle에 달려있는 device를 삭제하고 해당 bus도 삭제 한다.
 *          2) 새로 부여 받은 i2c handle을 반환한다.
 * @param old_handle 기존에 사용하던 handle
 * @return i2c_master_bus_handle_t  새로 만들어진 i2c_handle
 */
i2c_master_bus_handle_t i2c_bus_hardware_clear(i2c_master_bus_handle_t old_handle) {

    i2c_master_bus_handle_t handle ={};
    
    // 1. 기존 장치 및 버스 제거 (이미 할당된 경우)
    if (imu_handle[0])  i2c_master_bus_rm_device(imu_handle[0]);  //icm20948
    if (imu_handle[1])  i2c_master_bus_rm_device(imu_handle[1]);  //icm20948
    if (mag_handle[0])  i2c_master_bus_rm_device(mag_handle[0]);  //ist8310
    if (mag_handle[1])  i2c_master_bus_rm_device(mag_handle[1]);  //ak09916
    if (cbmp388_main.get_handle()) i2c_master_bus_rm_device(cbmp388_main.get_handle());    //bmp388
    if (cbmp388_sub.get_handle()) i2c_master_bus_rm_device(cbmp388_sub.get_handle());    //bmp388

    if (old_handle) {
        i2c_del_master_bus(old_handle);
    }

    // 2. SCL 핀 강제 토글 (Bus Clear)
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << I2C_SCL) | (1ULL << I2C_SDA);
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD; // Open-Drain 필수
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // SCL 9번 토글하여 고착된 SDA 해제
    for (int i = 0; i < 9; i++) {
        gpio_set_level(I2C_SCL, 0);
        esp_rom_delay_us(5);
        gpio_set_level(I2C_SCL, 1);
        esp_rom_delay_us(5);
    }

    // 3. I2C 버스 다시 초기화
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = I2C_SDA;
    bus_cfg.scl_io_num = I2C_SCL;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7; // 노이즈 필터링 강화
    bus_cfg.flags.enable_internal_pullup = true;

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &handle);
    if(ret ==ESP_OK) return handle;
    else return NULL;
}


}