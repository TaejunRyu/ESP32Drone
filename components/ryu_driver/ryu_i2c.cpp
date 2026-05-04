#include "ryu_i2c.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>


namespace Driver{

const char* I2C::TAG = "I2C";

I2C::I2C(): _port(I2C_PORT),_port_sda(I2C_SDA),_port_scl(I2C_SCL), _bus_handle(nullptr),_initialized(false)
{
    ESP_LOGI(TAG,"Initializing I2C Driver...");
}

I2C::~I2C()
{
    deinitialize();
}

esp_err_t I2C::initialize()
{        
    if(_initialized){
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = _port;
    bus_cfg.sda_io_num = _port_sda;
    bus_cfg.scl_io_num = _port_scl;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.intr_priority = 1;
    bus_cfg.flags.enable_internal_pullup = true;;
    bus_cfg.flags.allow_pd = false;

    // 내부에서 생성한 bus_handle의 주소를 전달
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &_bus_handle);

    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "Initialize Failed.");
        return ret; // 실패 시 NULL 반환
    }

    _initialized = true;
    ESP_LOGI(TAG, "Initialized successfully.");
    return ret; // 생성된 핸들 반환
    
}

void I2C::deinitialize()
{
    if (_bus_handle) {
        // ESP-IDF 5.x 이상: 내부 장치를 rm_device 안 해도 
        // 버스 삭제 시 리소스가 강제 해제되지만, 에러 로그 방지를 위해 reset 고려
        i2c_del_master_bus(_bus_handle); 
        _bus_handle = nullptr; 
    }

    // 2. SCL/SDA 핀 제어권 획득 및 강제 토글 (Bus Clear)
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << I2C_SCL) | (1ULL << I2C_SDA);
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD; 
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // SCL 9번 토글 (슬레이브 장치 리셋 유도)
    for (int i = 0; i < 9; i++) {
        gpio_set_level(I2C_SCL, 0);
        esp_rom_delay_us(5);
        gpio_set_level(I2C_SCL, 1);
        esp_rom_delay_us(5);
    }

    _initialized = false;
    ESP_LOGI(TAG, "I2C Bus & GPIO recovered and deinitialized.");
}


void I2C::scan_bus()
{
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
            esp_err_t ret = i2c_master_probe(_bus_handle, addr, pdMS_TO_TICKS(10));
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
}