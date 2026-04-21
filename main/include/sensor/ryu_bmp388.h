#pragma once

#include <math.h>
#include <driver/i2c_master.h>
#include <esp_log.h>
#include "ryu_config.h"

class CBMP388{
    private:
        uint8_t REG_ID        =   0x00;
        uint8_t REG_DATA      =   0x04; // Pressure(3), Temp(3) 연속
        uint8_t REG_PWR_CTRL  =   0x1B;
        uint8_t REG_CALIB     =   0x31;
        uint8_t STATUS        =   0x03;
        i2c_master_dev_handle_t handle = {};
        //esp_err_t last_error_code;  // 마지막 I2C 작업의 에러 코드 저장용
        //보정계수(시작)
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
        //보정계수(끝)
        // 보정계수와 미리계산 될것(한번만 하면 되는것들)
        float _p1, _p2, _p3, _p4, _p5, _p6, _p7, _p8, _p9, _p10, _p11;

        // bmp388에서 읽은 raw data.
        uint32_t uncomp_temp;
        uint32_t uncomp_press;

        float ground_pressure   = 1013.25f; // 초기값, 보정 후 업데이트됨
        float current_alt       = 0.0f; // 현재 고도
        float filtered_alt      = 0.0f; // 고도 필터링용
        float last_altitude     = 0.0f;  // 이전 고도 저장용
        float climb_rate        = 0.0f; // 상승 속도  
        // 정상으로 읽은 이전값   현재 데이터가 잘못되면 이전값을 내어준다.      
        uint32_t adc_p_last, adc_t_last;
        esp_err_t  read_calib();
        bool  is_data_ready();
        void init_coefficients();
        std::tuple<esp_err_t, uint32_t, uint32_t> read_bmp388(); 
        std::tuple<esp_err_t, float> get_pressure();
    public:
        static constexpr uint8_t ADDR_VCC   =   0x77;
        static constexpr uint8_t ADDR_GND   =   0x76;
        CBMP388(){};
        CBMP388(i2c_master_bus_handle_t bus_handle, uint16_t dev_address){
            initialize(bus_handle,dev_address);
        };
        
        i2c_master_dev_handle_t get_handle(){return handle;};
        //esp_err_t get_last_error() const { return last_error_code; } // 마지막 에러 코드 반환
        esp_err_t initialize(i2c_master_bus_handle_t bus_handle, uint16_t dev_address);
        std::tuple<esp_err_t ,float> get_relative_altitude();
        std::tuple<esp_err_t ,float> calibrate_ground_pressure();
        float update_climb_rate();

};


