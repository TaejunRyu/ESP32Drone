#include "ryu_flight_task.h"

#include <esp_task_wdt.h>
#include <esp_log.h>
#include <esp_event.h>
#include <esp_err.h>
#include <nvs_flash.h>
#include <nvs.h>
#include "ryu_buzzer.h"
#include "ryu_motor.h"
#include "ryu_config.h"
#include "ryu_led.h"
#include "ryu_sensor_event.h"
#include "ryu_failsafe.h"

extern "C" {
	void app_main(void);
}

void watch_dog_initialize();
void check_memory_check();
void check_system_health_on_boot(void);
void __attribute__((weak)) esp_panic_handler_reboot(void);

static const char *TAG = "MAIN";



// --- 메인 함수 ---
void app_main(void) {
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "RYU-Drone v2.0 starting...");
    ESP_LOGI(TAG, "ESP32 MCU");
    //ESP_LOGI(TAG, "=========================================");
    
    check_system_health_on_boot();
    g_sys.system_status = SYS_STATE_BOOT;
   
    esp_log_level_set("*", ESP_LOG_VERBOSE);//(5): 모든 데이터 출력 (매우 상세)
    
    auto& led = Driver::Led::get_instance();
    led.initialize();
    led.on();
 
   // 1. 기초 인프라 초기화 (가장 먼저)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    // 2. 기본 시스템 이벤트 루프 생성 (이벤트 시스템 사용을 위해 필수)
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    auto& failsafe      = Service::FailSafe::get_instance();
    esp_err_t err = failsafe.initialize();
    if (err != ESP_OK){
        ESP_LOGI(TAG, "FailSafe Module Initialize Failed.");
    }else{
        failsafe.start_task();
    }

    watch_dog_initialize();

    auto& flight = Controller::Flight::get_instance();
    err = flight.initialize();
    if (err != ESP_OK){
        ESP_LOGE(TAG, "Fligth Initialize Failed: %s", esp_err_to_name(err));
    }else{
        BaseType_t ret = flight.start_task();
        if (ret != pdPASS){
            ESP_LOGE(TAG, "Fligth Task Staring... Failed.");
        }
    }
    check_memory_check();    
    while (true) {
        led.toggle();
        vTaskDelay(pdMS_TO_TICKS(500)); // 메인 태스크가 종료되지 않게 붙잡음
    }
}

//esp_log_level_set("*", ESP_LOG_NONE);   //(0): 로그 끔
//esp_log_level_set("*", ESP_LOG_ERROR);  //(1): 치명적 오류만 출력
//esp_log_level_set("*", ESP_LOG_WARN);   //(2): 경고까지 출력
//esp_log_level_set("*", ESP_LOG_INFO);   //(3): 일반 정보까지 출력 (기본값)
//esp_log_level_set("*", ESP_LOG_DEBUG);  //(4): 디버그용 상세 정보
//esp_log_level_set("*", ESP_LOG_VERBOSE);//(5): 모든 데이터 출력 (매우 상세)
//esp_log_level_set("Mavlink", ESP_LOG_VERBOSE);//(5): 모든 데이터 출력 (매우 상세)
 

/**
 * @brief // 워치독 초기화 (5초 설정: 센서 재초기화 시간을 고려하여 넉넉히)
 * 
 */
void watch_dog_initialize(){
	esp_err_t ret;
	esp_task_wdt_config_t twdt_config = {
		.timeout_ms = 5000, 
		.idle_core_mask = (1 << 0) | (1 << 1), // 양쪽 코어 IDLE 감시
		.trigger_panic = true,                 // 문제시 시스템 리셋(Panic)
	};
	ret = esp_task_wdt_reconfigure(&twdt_config);
	if (ret ==ESP_OK){
		ESP_LOGI(TAG, "Successfully registered with Watch Dog.");
	}else{
		ESP_LOGI(TAG, "Failed to register as Watch Dog.");
	
	}
}

/**
 * @brief 
 * 		이 함수는 Panic이 발생했을 때 CPU가 리셋되기 직전에 호출됩니다.
 */
void __attribute__((weak)) esp_panic_handler_reboot(void) {
    // 1. 모든 모터 PWM 핀을 LOW로 강제 고정 (하드웨어적 정지)
    Driver::Motor::get_instance().stop_all_motors();
    // 2. 필요하다면 부저(Driverer)를 울려 경고
    Driver::Buzzer::get_instance().sound_error();

    // 3. 잠시 대기 (모터가 확실히 멈출 시간)
    for (int i = 0; i < 1000000; i++) {
    	__asm__ __volatile__("nop"); // 아무것도 안 하는 명령어를 강제로 삽입
	}
}

void check_memory_check(){
    ESP_LOGI(TAG, "========== System Health Check ==========",);
    ESP_LOGI(TAG, "Free heap: %u bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Min free heap: %u bytes", esp_get_minimum_free_heap_size());
    ESP_LOGI(TAG, "Free PSRAM: %u bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "Total PSRAM: %u bytes", heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "=========================================");
}

/**
 * @brief 
 * 		1. 부팅시 실행
 * 		2. 이전의 리부팅의 원인을 출력
 * 
 */
void check_system_health_on_boot(void) {
    esp_reset_reason_t reason = esp_reset_reason();                   
    ESP_LOGI(TAG, "======= Previous Reset Reason: %d ========", reason);
    switch (reason) {
        case ESP_RST_POWERON:
            ESP_LOGI(TAG, "Cold Boot: Power applied.");
            break;
        case ESP_RST_PANIC:
            ESP_LOGE(TAG, "Software Panic! Check for null pointers or stack overflow.");
            // 비상 상황 로그를 NVS에 저장하거나 부저로 알림
            break;
        case ESP_RST_TASK_WDT:
            ESP_LOGE(TAG, "Task WDT! Control Loop or Error Manager hung.");
            // PID 루프가 너무 무겁거나 I2C 대기 시간이 너무 긴지 확인 필요
            break;
        case ESP_RST_BROWNOUT:
            ESP_LOGW(TAG, "Brownout detected! Check battery voltage and capacitors.");
            // 전압 부족으로 꺼졌으므로 즉시 이륙 금지
            break;
        default:
            ESP_LOGW(TAG, "Reset occurred due to reason: %d", reason);
            break;
    }
    check_memory_check();
}

