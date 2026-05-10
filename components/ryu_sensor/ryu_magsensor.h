/**
 * @file ryu_magsensor.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-05-10
 * 
 * @copyright Copyright (c) 2026
 *      1. ist831 : main sensor
 *      2. ak09916: sub  sensor
 *      3. 1. 과 2.를 이용하여 작동 가능한 상태의 sensor의 데이터를 출력한다.
 */

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

namespace Sensor{

class  ManageMag{
    private:
        ManageMag() = default; 
        ~ManageMag() = default;
        static constexpr const char* TAG = "ManageMag";
        
        size_t _active_index = 0;
        size_t _err_count = 0;
        size_t _err_continue_count = 0;
        float  _avr_mag[3] = {};

        // 두센서가 문제 발생하여 이벤트로 보낸후 일정한 count후에 복구 이벤트를 보낸다.
        uint32_t _waiting_count =0;

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