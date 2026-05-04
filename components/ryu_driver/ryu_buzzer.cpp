#include "ryu_buzzer.h"

#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <esp_log.h>

namespace Driver
{

const char* Buzzer::TAG = "Buzzer";


Buzzer::Buzzer(){
    ESP_LOGI(TAG,"Initializing Buzzer Driver...");
}

Buzzer::~Buzzer(){
    deinitialize();
}


esp_err_t Buzzer::initialize()
{
    if (_initialized) {
        return ESP_OK;
    }
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    ledc_timer.duty_resolution  = LEDC_TIMER_10_BIT;
    ledc_timer.timer_num        = LEDC_TIMER_0;
    ledc_timer.freq_hz          = 2000;  // 초기 주파수 2kHz
    ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    
    esp_err_t err =ledc_timer_config(&ledc_timer);
    if (err != ESP_OK){
        return err;
    }
    ledc_channel_config_t ledc_channel = {};
    ledc_channel.gpio_num       = Driver::Buzzer::BUZZER_GPIO;
    ledc_channel.speed_mode     = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel        = LEDC_CHANNEL_0;
    //ledc_channel.intr_type      = LEDC_INTR_DISABLE;
    ledc_channel.timer_sel      = LEDC_TIMER_0;
    ledc_channel.duty           = 0; // 처음엔 소리 안 나게
    ledc_channel.hpoint         = 0;

    err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK){
        return err;
    }
    ESP_LOGI(TAG,"Initialized successfully.");
    _initialized = true;
    return err;
}

void Buzzer::deinitialize()
{
    if (!_initialized) return;


    _initialized = false;
    ESP_LOGD(TAG,"Buzzer deintialized.");
}

void Buzzer::play_tone(uint32_t freq_hz, uint32_t duration_ms)
{
        if (freq_hz > 0) {
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, freq_hz);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512); // 50% 듀티 ====>  0 ~ 512  소리 조절
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0); // 소리 끔
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
}
// 성공
void Buzzer::sound_success() {
    play_tone(523, 100); // C5
    play_tone(659, 100); // E5
    play_tone(783, 200); // G5
    play_tone(0, 10); 
}


void Buzzer::sound_error() {
    play_tone(783, 100); // G5
    play_tone(698, 100); // F5
    play_tone(659, 300); // E5
    play_tone(0, 10); 
}

//배터리 부족 경고 (Low Battery)
void Buzzer::sound_low_battery() {
    for(int i=0; i<3; i++) {
        play_tone(1568, 100); // 높은 솔 (G6)
        play_tone(0, 50);      // 잠시 쉼
    }
    play_tone(0, 10); 
}

//미션 완료 / 목표 지점 도착 (Waypoint Reached)
void Buzzer::sound_mission_complete() {
    play_tone(880, 150); // 라 (A5)
    play_tone(1174, 150); // 높은 레 (D6)
    play_tone(1318, 300); // 높은 미 (E6)
    play_tone(0, 10); 
}

void Buzzer::sound_system_start(){
    play_tone(523, 150); // C5 (도)
    play_tone(659, 150); // E5 (미)
    play_tone(783, 150); // G5 (솔)
    play_tone(1046, 300); // C6 (높은 도)
    play_tone(0, 10); 
}

// 1. 긴급 위험 경고 (Obstacle / Critical Error)
// 날카로운 사이렌 소리로 즉시 주의를 끕니다.
void Buzzer::sound_emergency() {
    for(int i=0; i<2; i++) {
        play_tone(1046, 150); // C6
        play_tone(1396, 150); // F6
    }
    play_tone(0, 10);
}

// 2. 버튼 입력 / 확인 (Click / Soft Confirm)
// 짧고 가벼운 소리로 사용자 조작에 피드백을 줍니다.
void Buzzer::sound_click() {
    play_tone(987, 50); // B5
    play_tone(0, 10);
}

// 3. 시스템 종료 (Power Down)
// 시스템 시작과 반대로 음이 내려가며 종료를 알립니다.
void Buzzer::sound_system_off() {
    play_tone(1046, 150); // C6
    play_tone(783, 150);  // G5
    play_tone(523, 300);  // C5
    play_tone(0, 10);
}

// 4. 데이터 전송 / 처리 중 (Processing / Thinking)
// 귀여운 비프음으로 무언가 작업 중임을 나타냅니다.
void Buzzer::sound_processing() {
    play_tone(880, 80);  // A5
    play_tone(1174, 80); // D6
    play_tone(880, 80);  // A5
    play_tone(0, 10);
}


//깜짝 놀람 (Surprise / Oh-Oh!): "어라?" 하는 느낌으로 음이 급격히 올라갔다 내려옵니다
void Buzzer::sound_surprise() {
    play_tone(1046, 50); // C6
    play_tone(2093, 150); // C7
    play_tone(0, 10);
}
//슬픔 / 실망 (Sad / Disappointed): 기운이 빠지는 듯 음이 길게 떨어집니다
void Buzzer::sound_sad() {
    play_tone(659, 200); // E5
    play_tone(523, 200); // C5
    play_tone(440, 400); // A4
    play_tone(0, 10);
}

//궁금함 / 질문 (Question / Confused): 끝음을 올려 질문하는 듯한 뉘앙스를 줍니다
void Buzzer::sound_question() {
    play_tone(783, 100); // G5
    play_tone(1046, 200); // C6
    play_tone(0, 10);
}

//장애물 근접 (Obstacle Proximity): 거리가 가까워질수록 반복 속도를 높여 사용할 수 있는 단일 비프음입니다.
void Buzzer::sound_proximity_alert() {
    play_tone(1318, 50); // E6 (매우 짧고 날카로운 소리)
    play_tone(0, 50); 
}

//스캔 중 (Scanning / Radar): 마치 레이더가 돌아가는 듯한 일정한 리듬의 소리입니다
void Buzzer::sound_scanning() {
    for(int i=0; i<3; i++) {
        play_tone(880, 50); // A5
        play_tone(0, 150);
    }
}


// 5. 연결 성공 (Connected / Paired)
// 블루투스나 Wi-Fi가 연결되었을 때 적합한 경쾌한 상승음입니다.
void Buzzer::sound_connected() {
    play_tone(523, 100); // C5
    play_tone(1046, 100); // C6
    play_tone(0, 10);
}


//연결 해제 (Disconnected): 연결 성공(sound_connected)과 반대되는 하강음입니다
void Buzzer::sound_disconnected() {
    play_tone(1046, 100); // C6
    play_tone(523, 150);  // C5
    play_tone(0, 10);
}


}
