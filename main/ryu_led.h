#pragma once

#include <driver/gpio.h>
#include <esp_log.h>

namespace Driver {

/**
 * @brief 내장 LED 제어를 위한 싱글톤 클래스 (GPIO_NUM_2)
 */
class Led {
    private:
        Led() = default; 
        ~Led() = default;
        static constexpr const char* TAG = "LedDriver";
    public:
        static Led& get_instance() {
            static Led instance; 
            return instance;
        }
        Led(const Led&) = delete;
        Led& operator=(const Led&) = delete;
        Led(Led&&) = delete;
        Led& operator=(Led&&) = delete;

        /**
         * @brief GPIO 초기화
         * @return esp_err_t (ESP_OK: 성공, ESP_ERR_INVALID_ARG: 핀 오류 등)
         */
        esp_err_t initialize() {
            if (_initialized) return ESP_OK;

            gpio_config_t io_conf = {
                .pin_bit_mask = (1ULL << _gpio_pin),
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE,
            };

            esp_err_t err = gpio_config(&io_conf);
            if (err == ESP_OK) {
                _initialized = true;
                ESP_LOGI(TAG, "LED initialized on GPIO %d", _gpio_pin);
            }
            return err;
        }

        void on() {
            if (_initialized) gpio_set_level(_gpio_pin, 1);
        }

        void off() {
            if (_initialized) gpio_set_level(_gpio_pin, 0);
        }

        void toggle() {
            if (_initialized) {
                _state = !_state;
                gpio_set_level(_gpio_pin, _state);
            }
        }

        bool is_initialized() const { return _initialized; }

    private:
        const gpio_num_t _gpio_pin = GPIO_NUM_2;
        bool _initialized = false;
        bool _state = false;
};

} // namespace Driver
