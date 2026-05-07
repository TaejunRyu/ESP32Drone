#pragma once

#include <tuple>
#include <array>
#include <string>
#include <esp_err.h>

namespace Sensor{
    class IST8310;
    class AK09916;
}

namespace Interface{
    enum class BusType;
}

namespace Service{

class  ManageMag{
    private:
        ManageMag() = default; 
        ~ManageMag() = default;
        static constexpr const char* TAG = "ManageMag";
        
        size_t active_index = 0;
        size_t err_count = 0;
        size_t err_continue_count = 0;
        float avr_mag[3] = {};

        // 2개의 센서 값의 차이 만큼을 보정.
        static inline constexpr float diff_x =  0.2784f;
        static inline constexpr float diff_y = -0.1175f;
        static inline constexpr float diff_z = -0.1285;

    public:
        static ManageMag& get_instance() {
            static ManageMag instance; 
            return instance;
        }
        ManageMag(const ManageMag&) = delete;
        ManageMag& operator=(const ManageMag&) = delete;
        ManageMag(ManageMag&&) = delete;
        ManageMag& operator=(ManageMag&&) = delete;

        // 실행과정에서 복구가 되면 
        bool is_fault_posted = false; // 이벤트 중복 발행 방지
        
        esp_err_t initialize();
        bool is_initialized(){return _initialized;};
        std::tuple<esp_err_t, std::array<float, 3>> Managed_read_with_offset();
        Interface::BusType get_bus_type(){return _busType;};

    private:
        Interface::BusType _busType;
        bool  _initialized = false;
};

}// namespace Service