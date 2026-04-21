#include "ryu_74hc138.h"

void select_cs_device(uint8_t index) {
    // index의 각 비트를 추출하여 GPIO 상태 결정 (Active Low 디코더 기준)
    gpio_set_level(DEC_A, (index >> 0) & 0x01); // 2^0 자리
    gpio_set_level(DEC_B, (index >> 1) & 0x01); // 2^1 자리
    gpio_set_level(DEC_C, (index >> 2) & 0x01); // 2^2 자리
}


void init_decoder_pins() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DEC_A) | (1ULL << DEC_B) | (1ULL << DEC_C),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // 초기 상태: 아무 센서도 선택하지 않음 (Y7 선택)
    select_cs_device(NO_SELECT);
}


enum {
    CS_ICM_1 = 0, // Y0
    CS_BMP_1 = 1, // Y1
    CS_ICM_2 = 2, // Y2
    CS_BMP_2 = 3, // Y3
    CS_NONE  = 7  // Y7 (아무것도 선택 안 함)
};


// // 통신 직전 호출되는 콜백 (IRAM_ATTR 필수)
// void IRAM_ATTR spi_pre_transfer_callback(spi_transaction_t *t) {
//     uint32_t cs_id = (uint32_t)t->user; // t->user에 담긴 인덱스 사용
//     gpio_set_level(GPIO_NUM_12, (cs_id >> 0) & 0x01);
//     gpio_set_level(GPIO_NUM_13, (cs_id >> 1) & 0x01);
//     gpio_set_level(GPIO_NUM_14, (cs_id >> 2) & 0x01);
// }

