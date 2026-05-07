#pragma once
#include <array>
#include <string>
#include <tuple>
#include <esp_err.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
 
// 인터페이스 전방 선언
namespace Interface {
    class BusInterface;
}

namespace Sensor {

class ICM20948 {
private:
    // private 생성자: Main()과 Sub() 인스턴스 생성 시 사용
    // 1. 누락된 생성자 구현부 추가
    ICM20948(std::string n) :_name(n),_isAlive(true) {}
    ~ICM20948() = default;

    static constexpr const char* TAG = "ICM20948";

    // 정적 인스턴스 (Main/Sub)
    static ICM20948 mainInstance;
    static ICM20948 subInstance;

public:
    // 싱글톤 및 인스턴스 접근자
    static ICM20948& Main() { return mainInstance; }
    static ICM20948& Sub() { return subInstance; }

    // 삭제된 생성자들 (복사/이동 방지)
    ICM20948(const ICM20948&) = delete;
    ICM20948& operator=(const ICM20948&) = delete;

    // 인터페이스 주입 (핵심!)
    void set_bus(Interface::BusInterface* bus) { _bus = bus; }
    Interface::BusInterface* get_bus(){ return _bus;};    
    // 상태 관리
    void setStatus(bool status) { _isAlive = status; }
    bool getStatus() { return _isAlive; }
    bool is_initialized() { return _initialized; }

    // I2C && SPI 레지스터 및 상수 정의
    static inline constexpr uint8_t ADDR_GND = 0x68;
    static inline constexpr uint8_t ADDR_VCC = 0x69;
    static inline constexpr uint8_t REG_BANK_SEL = 0x7F;
    static inline constexpr uint8_t B0_WHO_AM_I = 0x00;
    static inline constexpr uint8_t B0_PWR_MGMT_1 = 0x06;
    static inline constexpr uint8_t B0_ACCEL_XOUT_H = 0x2D;
    static inline constexpr uint8_t B0_GYRO_XOUT_H = 0x33;
    static inline constexpr uint8_t B0_INT_PIN_CFG = 0x0F;
    static inline constexpr uint8_t B0_USER_CTRL = 0x03;
    static inline constexpr uint8_t B2_GYRO_CONFIG_1 = 0x01;
    static inline constexpr uint8_t B2_ACCEL_CONFIG = 0x14;


    // SPI BYPASS기능을위해... 기존 레지스터 정의 아래에 추가 ...
    static inline constexpr uint8_t B3_I2C_MST_CTRL = 0x01;
    static inline constexpr uint8_t B3_I2C_SLV0_ADDR = 0x03;
    static inline constexpr uint8_t B3_I2C_SLV0_REG = 0x04;
    static inline constexpr uint8_t B3_I2C_SLV0_CTRL = 0x05;
    static inline constexpr uint8_t B0_EXT_SLV_SENS_DATA_00 = 0x3B; // SPI 모드에서 지자계 데이터가 들어오는 시작점


    esp_err_t setup_i2c_interface(i2c_master_bus_handle_t bus_handle, uint16_t addr);
    // 주요 기능 함수
    void icm20948_select_bank(uint8_t bank);
    esp_err_t enable_mag_bypass();
    void calibrate();
    esp_err_t initialize();
    esp_err_t deinitialize();

    // 데이터 읽기
    std::tuple<esp_err_t, std::array<float, 3>, std::array<float, 3>> read_raw_data();
    std::tuple<esp_err_t, std::array<float, 3>, std::array<float, 3>> read_with_offset();    
    // 통합 관리 함수 (예: Main/Sub 데이터 동시 처리 등)
    std::tuple<esp_err_t, std::array<float, 3>, std::array<float, 3>> Managed_read_with_offset();
    std::array<float, 3> get_mag() { return _mag_data; } // 지자계 데이터 접근자
private:
    Interface::BusInterface* _bus = nullptr; // 하드웨어 추상화 레이어
    std::array<float, 3> _mag_data = {0.0f, 0.0f, 0.0f}; // 최신 지자계 데이터 보관
    
    std::array<float, 3> _offset_acc = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> _offset_gyro = {0.0f, 0.0f, 0.0f};
    
    uint8_t _current_bank = 0;
    std::string _name {};
    
    bool _initialized = false;
    bool _calibration = false;
    bool _isAlive = false;
};

} // namespace Sensor
