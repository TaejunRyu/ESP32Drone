#include <esp_system.h>
#include <esp_task_wdt.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include "ryu_config.h"
#include "ryu_buzzer.h"
#include "ryu_servo.h"


// 현재 사용하는 센서 이름을 상수로 정의하여 코드 가독성 향상
inline constexpr char IMU_NAME_MAIN[]     ="ICM20948";
inline constexpr char IMU_NAME_SUB[]      ="ICM20948";
inline constexpr char MAG_NAME_MAIN[]     ="IST8310";
inline constexpr char MAG_NAME_SUB[]      ="AK09916";
inline constexpr char BARO_NAME_MAIN[]    ="BMP388";
inline constexpr char BARO_NAME_SUB[]     ="BMP388";


extern "C" {
	void app_main(void);
}

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
		ESP_LOGI("MAIN", "Watch Dog에 등록 성공.");
	}else{
		ESP_LOGI("MAIN", "Watch Dog에 등록 실패.");
	
	}
}

/**
 * @brief log 제한 
 * 
 */
void log_information_control(){
    // esp_log_level_set("*", ESP_LOG_NONE);   //(0): 로그 끔
    // esp_log_level_set("*", ESP_LOG_ERROR);  //(1): 치명적 오류만 출력
    // esp_log_level_set("*", ESP_LOG_WARN);   //(2): 경고까지 출력
     esp_log_level_set("*", ESP_LOG_INFO);   //(3): 일반 정보까지 출력 (기본값)
    // esp_log_level_set("*", ESP_LOG_DEBUG);  //(4): 디버그용 상세 정보
    // esp_log_level_set("*", ESP_LOG_VERBOSE);//(5): 모든 데이터 출력 (매우 상세)

    // esp_log_level_set("PROCESS_CMD_LONG", ESP_LOG_NONE);   //(0): 로그 
    // esp_log_level_set("TELETETRY", ESP_LOG_NONE);   //(0): 로그 
    // esp_log_level_set("MAIN", ESP_LOG_NONE);   //(0): 로그 
    // esp_log_level_set("GPS", ESP_LOG_NONE);   //(0): 로그 
}


/**
 * @brief 
 * 		이 함수는 Panic이 발생했을 때 CPU가 리셋되기 직전에 호출됩니다.
 * 		IRAM_ATTR을 붙여서 플래시 메모리가 아닌 RAM에서 즉시 실행되도록 합니다.
 */
void __attribute__((weak)) esp_panic_handler_reboot(void) {
    // 1. 모든 모터 PWM 핀을 LOW로 강제 고정 (하드웨어적 정지)
    // 예: GPIO 12, 13, 14, 15가 모터 핀이라면
    gpio_set_level((gpio_num_t)SERVO::MOTOR_FRONT_LEFT, 0);
    gpio_set_level((gpio_num_t)SERVO::MOTOR_FRONT_RIGHT, 0);
    gpio_set_level((gpio_num_t)SERVO::MOTOR_REAR_LEFT, 0);
    gpio_set_level((gpio_num_t)SERVO::MOTOR_REAR_RIGHT, 0);

    // 2. 필요하다면 부저(Driverer)를 울려 경고
    Driver::Buzzer::get_instance().sound_error();

    // 3. 잠시 대기 (모터가 확실히 멈출 시간)
    for (int i = 0; i < 1000000; i++) {
    	__asm__ __volatile__("nop"); // 아무것도 안 하는 명령어를 강제로 삽입
	}
}

/**
 * @brief 
 * 		1. 부팅시 실행
 * 		2. 이전의 리부팅의 원인을 출력
 * 
 */
void check_system_health_on_boot() {
    esp_reset_reason_t reason = esp_reset_reason();

    ESP_LOGI("SYSTEM", "========== Previous Reset Reason: %d ==========", reason);

    switch (reason) {
        case ESP_RST_POWERON:
            ESP_LOGI("SYSTEM", "Cold Boot: Power applied.");
            break;

        case ESP_RST_PANIC:
            ESP_LOGE("SYSTEM", "Software Panic! Check for null pointers or stack overflow.");
            // 비상 상황 로그를 NVS에 저장하거나 부저로 알림
            break;

        case ESP_RST_TASK_WDT:
            ESP_LOGE("SYSTEM", "Task WDT! Control Loop or Error Manager hung.");
            // PID 루프가 너무 무겁거나 I2C 대기 시간이 너무 긴지 확인 필요
            break;

        case ESP_RST_BROWNOUT:
            ESP_LOGW("SYSTEM", "Brownout detected! Check battery voltage and capacitors.");
            // 전압 부족으로 꺼졌으므로 즉시 이륙 금지
            break;

        default:
            ESP_LOGW("SYSTEM", "Reset occurred due to reason: %d", reason);
            break;
    }
}

