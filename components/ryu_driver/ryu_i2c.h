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

        i2c_master_bus_handle_t get_bus_handle(){return _bus_handle;};
        i2c_master_bus_handle_t initialize(i2c_port_num_t port, gpio_num_t port_sda, gpio_num_t port_scl);
        i2c_master_bus_handle_t initialize();
        void deinitialize();
        void scan_bus();
        

    private:
        i2c_port_num_t _port;
        gpio_num_t _port_sda;
        gpio_num_t _port_scl;

        i2c_master_bus_handle_t _bus_handle;        
        bool _initialized;
        static const char* TAG;
};


}