#pragma once

//#define LOG_LOCAL_LEVEL ESP_LOG_NONE  // 이 파일의 모든 로그를 끔

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <driver/gpio.h>

namespace  GPS
{

inline constexpr uint32_t   GPS_UART_BAUD_RATE = 115200;
inline constexpr gpio_num_t GPS_RX      = GPIO_NUM_16;  // UART1 
inline constexpr gpio_num_t GPS_TX      = GPIO_NUM_17;

extern SemaphoreHandle_t xGpsMutex;

    //=====ubx 바이너리 프로토콜======= start
#pragma pack(push, 1)
typedef struct {
    uint32_t iTOW;       // GPS 시간 (ms)
    uint16_t year;       // 년
    uint8_t  month;      // 월
    uint8_t  day;        // 일
    uint8_t  hour;       // 시
    uint8_t  min;        // 분
    uint8_t  sec;        // 초
    uint8_t  valid;      // 유효성 플래그( 시간과 날자의 유효성)
    uint32_t tAcc;       // 시간 정확도
    int32_t  nano;       // 나노초
    uint8_t  fixType;    // GNSS Fix 타입 (0:No Fix, 3:3D Fix 등)
    uint8_t  flags;      // 수정 플래그
    uint8_t  flags2;     // 추가 플래그
    uint8_t  numSV;      // 사용 중인 위성 수 (중요!)
    int32_t  lon;        // 경도 (1e-7 deg 단위) -> 실제값 = lon * 1e-7
    int32_t  lat;        // 위도 (1e-7 deg 단위)
    int32_t  height;     // 타원체 고도 (mm)
    int32_t  hMSL;       // 해수면 고도 (mm)  ---- 현재고도
    uint32_t hAcc;       // 수평 정확도 (mm)
    uint32_t vAcc;       // 수직 정확도 (mm)
    int32_t  velN;       // 북향 속도 (mm/s)
    int32_t  velE;       // 동향 속도 (mm/s)
    int32_t  velD;       // 하향 속도 (mm/s)
    int32_t  gSpeed;     // 지표 속도 (mm/s)
    int32_t  headMot;    // 이동 방향 (1e-5 deg)
    uint32_t sAcc;       // 속도 정확도 (mm/s)
    uint32_t headAcc;    // 헤딩 정확도 (1e-5 deg)
    uint16_t pDOP;       // 위치 정밀도 저하율 (0.01 단위)
    uint8_t  reserved1[6]; // 예약 영역
    int32_t  headVeh;    // 차량 헤딩 (1e-5 deg)
    int16_t  magDec;     // 자기 편각 (1e-2 deg)
    uint16_t magAcc;     // 자기 편각 정확도
} /*__attribute__((packed))*/ ubx_nav_pvt_t;
#pragma pack(pop)

extern void initialize();
extern void gps_ubx_mode_task(void *pvParameters);

void calculate_ubx_checksum(uint8_t* data, int len, uint8_t* ck_a, uint8_t* ck_b);
uint8_t checkDataReliability(ubx_nav_pvt_t *pvt);


}


