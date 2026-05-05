#pragma once

#include <driver/i2c_master.h>
#include <driver/gpio.h>

namespace Driver{

class I2C{
    private:
        I2C();
    public:
        // 🌟 복사 생성자와 대입 연산자 비활성화 (싱글톤 복사 방지)
        I2C(const I2C&) = delete;
        I2C& operator=(const I2C&) = delete;
        ~I2C();
        // 싱글톤 인스턴스 접근 메서드
        // 🌟 get_instance() 메서드 구현
        static I2C& get_instance() {
            static I2C* instance = new I2C(); // 힙에 할당하여 소멸 순서 꼬임 방지
            return *instance;
        }

        static inline constexpr uint32_t       I2C_SPEED   = 400'000;
        static inline constexpr gpio_port_t    I2C_PORT    = gpio_port_t(0);
        static inline constexpr gpio_num_t     I2C_SDA     = GPIO_NUM_21; // 숫자에 직접 타입을 지정
        static inline constexpr gpio_num_t     I2C_SCL     = GPIO_NUM_22;


        i2c_master_bus_handle_t get_bus_handle(){return _bus_handle;};
        esp_err_t initialize();
        esp_err_t  deinitialize();
        void scan_bus();

    private:
        i2c_port_num_t _port = I2C_PORT;
        gpio_num_t _port_sda = I2C_SDA;
        gpio_num_t _port_scl = I2C_SCL;

        i2c_master_bus_handle_t _bus_handle = nullptr;        
        bool _initialized = false;
        static const char* TAG;
};


}