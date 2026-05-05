#pragma once

#include <array> 
#include <tuple>
#include <driver/i2c_master.h>
#include <esp_log.h>

namespace Sensor
{

class IST8310{
    private:
        IST8310();
    public:
            // 🌟 복사 생성자와 대입 연산자 비활성화 (싱글톤 복사 방지)
        IST8310(const IST8310&) = delete;
        IST8310& operator=(const IST8310&) = delete;
        ~IST8310();

        // 싱글톤 인스턴스 접근 메서드
        // 🌟 get_instance() 메서드 구현
        static IST8310& get_instance() {
            static IST8310* instance = new IST8310(); // 힙에 할당하여 소멸 순서 꼬임 방지
            return *instance;
        }  

        esp_err_t initialize();
        esp_err_t deinitialize();

        std::tuple<esp_err_t, std::array<float, 3>> read_raw_data();
        std::tuple<esp_err_t, std::array<float, 3>> read_with_offset();
        void calibrate_hard_iron();
        i2c_master_dev_handle_t get_dev_handle(){ return _dev_handle;};

    private:
        std::array<float, 3> last_valid_mag ={};

        static inline constexpr float FILTER_ALPHA = 0.4f; 
        // IST8310 레지스터 정의
        static inline constexpr uint8_t ADDR       =    0x0E; // 기본 주소 (ADR핀 상태에 따라 다를 수 있음)
        static inline constexpr uint8_t WHO_AM_I   =    0x00; // ID 확인용 (값: 0x10)
        static inline constexpr uint8_t STAT1      =    0x02; // 데이터 준비 상태 (Bit 0: DRDY)
        static inline constexpr uint8_t DATA_X_L   =    0x03; // 데이터 시작 (X-axis Low)
        static inline constexpr uint8_t CONTROL1   =    0x0A; // 모드 설정 (Single/Continuous)
        static inline constexpr uint8_t CONTROL2   =    0x0B; // 소프트 리셋 및 옵션
        static inline constexpr uint8_t AVGCNTL    =    0x41; // 평균 필터 설정 (노이즈 감소)
        static inline constexpr uint8_t PDCNTL     =    0x42; // Pulse Duration 제어
        static inline constexpr uint8_t CROSSAXIS1 =    0x48; // 
        static inline constexpr uint8_t CROSSAXIS2 =    0x49; // 
        // 센서 감도: 1320 LSB/Gauss (0.3 µT/LSB)

        static inline constexpr float SENSITIVITY   =  0.3f;   
        static inline constexpr float MAG_MAX_X     =  58.20f;
        static inline constexpr float MAG_MAX_Y     =  50.40f;
        static inline constexpr float MAG_MAX_Z     =  29.10f;

        static inline constexpr float MAG_MIN_X     =  -34.50f;
        static inline constexpr float MAG_MIN_Y     =  -41.70f;
        static inline constexpr float MAG_MIN_Z     =  -59.70f;

        static inline constexpr float MAG_OFFSET_X  =  11.85f;
        static inline constexpr float MAG_OFFSET_Y  =  4.35f;
        static inline constexpr float MAG_OFFSET_Z  =  -15.30f;

        static inline constexpr float SCALE_X       =  0.98f;
        static inline constexpr float SCALE_Y       =  0.99f;
        static inline constexpr float SCALE_Z       =  1.03f;

        i2c_master_bus_handle_t _bus_handle = nullptr;
        i2c_master_dev_handle_t _dev_handle = nullptr;
        bool _initialized = false;
        static const char* TAG;

};





}