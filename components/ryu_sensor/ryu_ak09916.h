#pragma once
#include <tuple>
#include <driver/i2c_master.h>
#include <esp_log.h>

namespace Sensor
{

class AK09916{
    private:
        AK09916() = default; 
        ~AK09916() = default;
        static constexpr const char* TAG = "AK09916";
    public:
        static AK09916& get_instance() {
            static AK09916 instance; 
            return instance;
        }
        AK09916(const AK09916&) = delete;
        AK09916& operator=(const AK09916&) = delete;
        AK09916(AK09916&&) = delete;
        AK09916& operator=(AK09916&&) = delete;
        static inline constexpr uint8_t ADDR       =    0x0C;
        static inline constexpr uint8_t STAT1      =    0x10;    // 데이터의 준비 체크

        static inline constexpr uint8_t HXL        =    0x11;// 데이터 시작 (X-axis Low)
        static inline constexpr uint8_t CNTL2      =    0x31;// 모드 설정 (100Hz 등)
        static inline constexpr uint8_t CNTL3      =    0x32;// 소프트 리셋
        static inline constexpr uint8_t WHO_AM_I   =    0x01;// ID 확인용 (값: 0x09)

        static inline constexpr float SCALE_X      =   0.91f;
        static inline constexpr float SCALE_Y      =   1.07f;
        static inline constexpr float SCALE_Z      =   1.04f;
        static inline constexpr float MAG_OFFSET_X =   91.50f;
        static inline constexpr float MAG_OFFSET_Y =   176.00f;
        static inline constexpr float MAG_OFFSET_Z =   -170.50f;

        esp_err_t  deinitialize();
        esp_err_t initialize();
        std::tuple<esp_err_t, std::array<float, 3>> read_data();
        std::tuple<esp_err_t, std::array<float, 3>> read_with_offset(); 
        std::tuple<esp_err_t,uint8_t> ready_data();
        void calibrate_hard_iron();
        i2c_master_dev_handle_t get_dev_handle(){return _dev_handle;};
        bool is_initialized(){return _initialized;};

    private:

       
        i2c_master_bus_handle_t _bus_handle = nullptr;
        i2c_master_dev_handle_t _dev_handle = nullptr;
        bool _initialized = false;
};

}