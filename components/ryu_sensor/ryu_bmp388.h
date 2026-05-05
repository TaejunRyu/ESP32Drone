#pragma once

#include <math.h>
#include <algorithm>

#include <string>
#include <driver/i2c_master.h>
#include <esp_log.h>

namespace Sensor{

class BMP388{
    private:
        BMP388() = default; 
        ~BMP388() = default;
        static constexpr const char* TAG = "BMP388";
        static BMP388 mainInstance;
        static BMP388 subInstance;
    public:

        BMP388(const BMP388&) = delete;
        BMP388& operator=(const BMP388&) = delete;
        BMP388(BMP388&&) = delete;
        BMP388& operator=(BMP388&&) = delete;

        // 2개의 내부 인스턴스에 접근하기 위한 인터페이스
        static BMP388& Main() { return mainInstance; }
        static BMP388& Sub() { return subInstance; }
        
        static inline constexpr uint8_t ADDR_VCC   =   0x77;
        static inline constexpr uint8_t ADDR_GND   =   0x76;

        static inline constexpr uint8_t REG_ID        =   0x00;
        static inline constexpr uint8_t REG_DATA      =   0x04; // Pressure(3), Temp(3) 연속
        static inline constexpr uint8_t REG_PWR_CTRL  =   0x1B;
        static inline constexpr uint8_t REG_CALIB     =   0x31;
        static inline constexpr uint8_t STATUS        =   0x03;

        void setStatus(bool status) { _isAlive = status; }
        bool getStatus() { return _isAlive; }
        esp_err_t initialize();
        esp_err_t deinitialize();
        i2c_master_dev_handle_t get_dev_handle(){return _dev_handle;};
        std::tuple<esp_err_t ,float> get_relative_altitude();
        std::tuple<esp_err_t ,float> calibrate_ground_pressure();        
        std::tuple<esp_err_t, float,float> Managed_get_relative_altitude();
        float get_climb_rate(){return _climb_rate;};

    private:
        //보정계수        
        struct {
            double      t_lin;
            uint16_t    t1; 
            uint16_t    t2;  
            int8_t      t3;
            int16_t     p1, p2; 
            int8_t      p3, p4;
            uint16_t    p5, p6;  
            int8_t      p7, p8;
            int16_t     p9; 
            int8_t      p10, p11;
        } _coef;

        // 보정계수와 미리계산 될것(한번만 하면 되는것들)
        float   _p1 = 0.0f, _p2 = 0.0f, _p3 = 0.0f, _p4 = 0.0f, _p5 = 0.0f, _p6 = 0.0f, 
                _p7 = 0.0f, _p8 = 0.0f, _p9 = 0.0f, _p10= 0.0f, _p11= 0.0f;

        // bmp388에서 읽은 raw data.
        uint32_t uncomp_temp  = 0;
        uint32_t uncomp_press = 0;

        float _ground_pressure   = 1013.25f; // 초기값, 보정 후 업데이트됨
        float _current_alt       = 0.0f; // 현재 고도
        float _filtered_alt      = 0.0f; // 고도 필터링용
        float _last_altitude     = 0.0f;  // 이전 고도 저장용
        float _climb_rate        = 0.0f; // 상승 속도  
        
        // 정상으로 읽은 이전값   현재 데이터가 잘못되면 이전값을 내어준다.      
        uint32_t    adc_p_last = 0, 
                    adc_t_last = 0;
                    
        float update_climb_rate();
        bool  is_data_ready();
        esp_err_t  read_calib();
        void init_coefficients();
        std::tuple<esp_err_t, uint32_t, uint32_t> read_bmp388(); 
        std::tuple<esp_err_t, float> get_pressure();


        i2c_master_bus_handle_t _bus_handle = nullptr;
        i2c_master_dev_handle_t _dev_handle = nullptr;
        uint16_t _dev_address {};
        bool _initialized = false;
        std::string _name {};
        bool _isAlive = false;
        // private 생성자: 외부에서 호출 불가
        BMP388(std::string n,uint16_t addr) : _dev_address(addr), _name(n), _isAlive(true) {}



    };

}


