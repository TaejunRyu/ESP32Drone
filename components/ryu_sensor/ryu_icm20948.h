#pragma once
#include <array>
#include <string>
#include <driver/i2c_master.h>
#include <esp_log.h>


namespace Sensor
{

// 1. ADO핀을 바꾸면 BMP388의 주소도 같이 다른번호로 바뀌는것 주의 

class ICM20948{
    public:
       // 복사 방지
        ICM20948(const ICM20948&) = delete;
        ICM20948& operator=(const ICM20948&) = delete;

        // 2개의 내부 인스턴스에 접근하기 위한 인터페이스
        static ICM20948& Main() { return mainInstance; }
        static ICM20948& Sub()  { return subInstance; }
        
        void setStatus(bool status) { _isAlive = status; }
        bool getStatus() { return _isAlive; }

        static inline constexpr uint8_t  ADDR_GND =  0x68; // AD0핀이 GND일 때 0x68
        static inline constexpr uint8_t  ADDR_VCC =  0x69; // AD0핀이 VCC일 때 0x69

        static inline constexpr uint8_t  REG_BANK_SEL     =  0x7F;
        static inline constexpr uint8_t  B0_WHO_AM_I      =  0x00; // 값: 0xEA
        static inline constexpr uint8_t  B0_PWR_MGMT_1    =  0x06;
        static inline constexpr uint8_t  B0_ACCEL_XOUT_H  =  0x2D; // 가속도 시작 (총 6바이트)
        static inline constexpr uint8_t  B0_GYRO_XOUT_H   =  0x33; // 자이로 시작 (총 6바이트)
        
        // 뱅크 및 레지스터 추가 정의
        static inline constexpr uint8_t  B0_INT_PIN_CFG   =  0x0F;
        static inline constexpr uint8_t  B0_USER_CTRL     =  0x03; 
        
        // 레지스터 정의 (Bank 2 기준)=
        static inline constexpr uint8_t  B2_GYRO_CONFIG_1 =  0x01;
        static inline constexpr uint8_t  B2_ACCEL_CONFIG  =  0x14;


        i2c_master_dev_handle_t get_dev_handle(){return _dev_handle;};
        void icm20948_select_bank(uint8_t bank);
        esp_err_t enable_mag_bypass();
        void calibrate();
        esp_err_t initialize();
        esp_err_t deinitialize();
        std::tuple<esp_err_t, std::array<float, 3>, std::array<float, 3>> read_raw_data();
        std::tuple<esp_err_t, std::array<float, 3>, std::array<float, 3>> read_with_offset();

        // Main module과 Sub module의 관리을 할수 있을까? 
        // 1. mainInstance,subInstance에서 데이터를 가져와보자.
        std::tuple<esp_err_t, std::array<float, 3>, std::array<float, 3>> Managed_read_with_offset();

    private:
        ICM20948();

        // offfset 저장
        std::array<float, 3> _offset_acc ={};
        std::array<float, 3> _offset_gyro ={};

        i2c_master_bus_handle_t _bus_handle = nullptr;
        i2c_master_dev_handle_t _dev_handle = nullptr;
        
        uint8_t _current_bank = 0;      // 현재 사용중인 뱅크 
        uint16_t _dev_address =0x00;      // ICM20948의 ADDR
        
        bool _initialized = false;;
        bool _calibration = false;
        std::string _name {};
        bool _isAlive = false;
        
        // private 생성자: 외부에서 호출 불가
        ICM20948(std::string n, uint16_t addr) :_dev_address(addr),_name(n),_isAlive(true) {}

        static ICM20948 mainInstance;
        static ICM20948 subInstance; 

        static const char *TAG;
};

}