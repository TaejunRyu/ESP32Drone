#pragma once

#include <stdint.h>
#include <driver/gpio.h>

namespace Driver{

    
class Buzzer{
    private:
        Buzzer() = default; 
        ~Buzzer() = default;
        static constexpr const char* TAG = "Buzzer";
    public:
        static Buzzer& get_instance() {
            static Buzzer instance; 
            return instance;
        }
        Buzzer(const Buzzer&) = delete;
        Buzzer& operator=(const Buzzer&) = delete;
        Buzzer(Buzzer&&) = delete;
        Buzzer& operator=(Buzzer&&) = delete;


    static inline constexpr gpio_num_t BUZZER_GPIO = GPIO_NUM_14;  // 부저가 연결된 GPIO 번호
 
    esp_err_t initialize();
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
};

}