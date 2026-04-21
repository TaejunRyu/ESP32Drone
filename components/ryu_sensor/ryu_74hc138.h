/**
 * @file 74hc138.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-04-04
 * 
 * @copyright Copyright (c) 2026
 * 
 * @details
 * 74HC138 3-to-8 디코더를 사용하여 SPI 센서의 Chip Select 핀을 제어하는 방법입니다.
 * 요약 및 주의사항동시 선택 불가: 디코더 특성상 한 번에 딱 하나의 센서만 선택됩니다.
 * 속도: gpio_set_level은 매우 빠르므로 SPI 통신 직전에 호출해도 지연 시간이 거의 없습니다.
 * Active Low: 74HC138은 선택된 출력이 0(Low)이 되므로, SPI 센서의 nCS 핀에 그대로 연결하면 됩니다.
 */
#include <driver/gpio.h>
#include "driver/spi_master.h"
#include <esp_attr.h>
// 디코더 입력 핀 정의 (예시 GPIO)
#define DEC_A  GPIO_NUM_12
#define DEC_B  GPIO_NUM_13
#define DEC_C  GPIO_NUM_14

// 디코더 출력이 연결된 센서 번호 정의
#define SENSOR_ICM_A  0  // 디코더 Y0
#define SENSOR_BMP_A  1  // 디코더 Y1
#define SENSOR_ICM_B  2  // 디코더 Y2
#define SENSOR_BMP_B  3  // 디코더 Y3
#define NO_SELECT     7  // 디코더 Y7 (아무것도 연결 안 함)

/**
 * 사용 예시
 * spi_device_interface_config_t devcfg = {
    .clock_speed_hz = 1*1000*1000,           // 1 MHz (테스트용)
    .mode = 0,                               // SPI mode 0
    .spics_io_num = -1,                      // ★ 중요: 하드웨어 CS 사용 안 함 (-1)
    .queue_size = 7,
    .pre_cb = NULL,                          // (선택사항) 아래 2번 설명 참조
    .post_cb = NULL,
};


// 1. 디코더로 센서 A(ICM) 선택
select_cs_device(SENSOR_ICM_A); 

// 2. SPI 통신 실행 (ESP-IDF 표준 함수)
spi_device_transmit(spi_handle, &transaction);

// 3. 통신 종료 후 디코더 해제 (필요 시)
select_cs_device(NO_SELECT); 

 * 
 */



 /**
  * 
  * // [콜백 함수 정의]
void IRAM_ATTR spi_pre_transfer_callback(spi_transaction_t *t) {
    // transaction의 .user 필드에 담긴 센서 번호를 읽어 디코더 조작
    uint32_t sensor_idx = (uint32_t)t->user;
    select_cs_device(sensor_idx);
}

// [설정 시 등록]
devcfg.pre_cb = spi_pre_transfer_callback;

// [실제 사용 시]
spi_transaction_t t;
memset(&t, 0, sizeof(t));
t.user = (void*)SENSOR_ICM_A; // ★ 이 트랜잭션이 센서 A용임을 명시
t.length = 8 * 3;             // 3바이트 읽기
t.rx_buffer = data_buffer;

spi_device_transmit(spi_handle, &t); // 콜백이 자동으로 디코더를 A로 맞춤

  * 
  */

extern void IRAM_ATTR spi_pre_transfer_callback(spi_transaction_t *t);
