#pragma once

#include <tuple>
#include <array>
#include <string>
#include <esp_err.h>

namespace Sensor{
    class IST8310;
    class AK09916;
}

namespace Service{

class  ManageMag{
    private:
        ManageMag() = default; 
        ~ManageMag() = default;
        static constexpr const char* TAG = "ManageMag";

    public:
        static ManageMag& get_instance() {
            static ManageMag instance; 
            return instance;
        }
        ManageMag(const ManageMag&) = delete;
        ManageMag& operator=(const ManageMag&) = delete;
        ManageMag(ManageMag&&) = delete;
        ManageMag& operator=(ManageMag&&) = delete;

        esp_err_t initialize();
        bool is_initialized(){return _initialized;};
        std::tuple<esp_err_t, std::array<float, 3>> Managed_read_with_offset();

    private:
        bool  _initialized = false;
};
}// namespace Service