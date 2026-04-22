#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include <freertos/timers.h>
#include <esp_timer.h>
#include <driver/gpio.h>

typedef struct {
    struct { float x, y, z; uint64_t ts; } imu; // 1kHz
    struct { float x, y, z; uint64_t ts; } mag; // 50Hz
    struct { float press, temp; uint64_t ts; } bmp; // 10Hz
} shared_data_t;


volatile shared_data_t g_sensor_hub;

TaskHandle_t xTaskToNotify = NULL;



// 1. IMU 전용 인터럽트 (가장 빠름 - PID 트리거)
static void IRAM_ATTR imu_drdy_handler(void* arg) {
    //read_imu_raw(&g_sensor_hub.imu.x, ...);
    g_sensor_hub.imu.ts = esp_timer_get_time();

    // IMU 데이터가 들어왔을 때만 PID 태스크를 깨움 (비행 제어의 기준)
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(xTaskToNotify, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) 
        portYIELD_FROM_ISR();
}

// 2. MAG 전용 인터럽트 (느림 - 데이터만 업데이트)
static void IRAM_ATTR mag_drdy_handler(void* arg) {
    //read_mag_raw(&g_sensor_hub.mag.x, ...);
    g_sensor_hub.mag.ts = esp_timer_get_time();
}

// 3. BMP 전용 인터럽트 (매우 느림 - 데이터만 업데이트)
static void IRAM_ATTR bmp_drdy_handler(void* arg) {
    //read_bmp_raw(&g_sensor_hub.bmp.press, ...);
    g_sensor_hub.bmp.ts = esp_timer_get_time();
}


void pid_task(void *pvParameters) {
    while (1) {
        // IMU 인터럽트가 올 때까지 대기
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 1. IMU 데이터는 항상 최신 (PID 직접 사용)
        float gx = g_sensor_hub.imu.x;

        // 2. MAG/BMP는 마지막 업데이트 타임을 체크하여 Mahony 필터에 반영
        // (예: 현재 시간과 ts의 차이가 일정 이상이면 데이터 무시 혹은 가중치 감소)
        uint64_t now = esp_timer_get_time();
        if (now - g_sensor_hub.mag.ts < 50000) { // 50ms 이내 데이터면 유효
            //apply_mag_correction(g_sensor_hub.mag.x, ...);
        }

        // 3. PID 계산 및 모터 출력
        //run_pid_logic();
    }
}
#define IMU_DRDY_PIN    (gpio_num_t)1
#define MAG_DRDY_PIN    (gpio_num_t)1
#define BMP_DRDY_PIN    (gpio_num_t)3

void register_gpio_isr_handle_add(){    
    // GPIO 인터럽트 서비스 설치
    gpio_install_isr_service(0);
    // 각 핀별 핸들러 등록 (Edge: Rising)
    gpio_isr_handler_add(IMU_DRDY_PIN, imu_drdy_handler, NULL);
    gpio_isr_handler_add(MAG_DRDY_PIN, mag_drdy_handler, NULL);
    gpio_isr_handler_add(BMP_DRDY_PIN, bmp_drdy_handler, NULL);
}
