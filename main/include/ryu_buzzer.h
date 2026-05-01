#pragma once

#include <stdint.h>

namespace BUZZ{

class Buzzer{
private:
    Buzzer();
public:
    // 🌟 복사 생성자와 대입 연산자 비활성화 (싱글톤 복사 방지)
    Buzzer(const Buzzer&) = delete;
    Buzzer& operator=(const Buzzer&) = delete;
    ~Buzzer();

    // 싱글톤 인스턴스 접근 메서드
    // 🌟 get_instance() 메서드 구현
    static Buzzer& get_instance() {
        static Buzzer* instance = new Buzzer(); // 힙에 할당하여 소멸 순서 꼬임 방지
        return *instance;
    }    
    void initialize();
    void deinitialize();
    void play_tone(uint32_t freq_hz, uint32_t duration_ms);
    void sound_success();
    void sound_error();
    void sound_low_battery();
    void sound_mission_complete();
    void sound_system_start();
    void sound_emergency();
    void sound_click();
    void sound_system_off();
    void sound_processing();
    void sound_surprise();
    void sound_sad();
    void sound_question();
    void sound_proximity_alert();
    void sound_scanning();
    void sound_connected();
    void sound_disconnected();

private:
    bool _initialized = false;
    static const char* TAG;
    
};

}