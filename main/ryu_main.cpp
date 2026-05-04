
#include "ryu_main.h"

#include "ryu_flight_task.h"

static const char *TAG = "MAIN";

// --- 메인 함수 ---
void app_main(void) {
    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "RYU-Drone v2.0 starting...");
    ESP_LOGI(TAG, "ESP32 MCU");
    ESP_LOGI(TAG, "======================================");
    // 시스템 시작시 여유를 준다.
    vTaskDelay(pdMS_TO_TICKS(1000));
    check_system_health_on_boot();

    log_information_control();

    watch_dog_initialize();

    { // 2번 포트 led 설정
        gpio_reset_pin(GPIO_NUM_2); // 핀 상태 초기화
        gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT); // 출력 모드로 설정       
        // buzzer 초기화및 fc의 시작을 알린다.
        gpio_set_level(GPIO_NUM_2,1);
    }
    
    auto& flight = Controller::Flight::get_instance();
    flight.initialize();
    flight.start_task();
    
    while (true) {
        static bool led_state = false;
        gpio_set_level(GPIO_NUM_2, (led_state = !led_state));
        vTaskDelay(pdMS_TO_TICKS(500)); // 메인 태스크가 종료되지 않게 붙잡음
    }
}
