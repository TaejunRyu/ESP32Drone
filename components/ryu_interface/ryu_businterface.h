#pragma once

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <driver/spi_master.h>
#include <driver/i2c_master.h>

namespace Interface {

enum class BusType { I2C, SPI }; // 통신 타입 정의

/**
 * 1. 통신 인터페이스 (부모)
 */
class BusInterface {
public:
    virtual ~BusInterface() = default;
    virtual BusType get_type() const = 0;      // 특이한 사항이 있을경우 _bus의 type을 알아서 대처한다. 
    virtual esp_err_t write(uint8_t reg, uint8_t data) = 0;
    virtual esp_err_t read(uint8_t reg, uint8_t* data, size_t len) = 0;
}; // 네임스페이스 안에서 인터페이스 정의 완료


/**
 * 2. SPI 통신 구현체
 */
class SPIBus : public BusInterface {
private:
    spi_device_handle_t _handle;

public:
    SPIBus(spi_device_handle_t handle) : _handle(handle) {}

    Interface::BusType get_type() const override { return Interface::BusType::SPI; }
    
    esp_err_t write(uint8_t reg, uint8_t data) override {
        spi_transaction_t t = {};
        t.flags = SPI_TRANS_USE_TXDATA; // 4바이트 이하 전송 시 성능 최적화
        t.addr = reg & 0x7F;           // SPI Write: MSB 0
        t.length = 8;                  // 데이터 8비트(1바이트)
        t.tx_data[0] = data;           // tx_data 배열의 첫 번째에 데이터 저장
        return spi_device_polling_transmit(_handle, &t);
    }

    esp_err_t read(uint8_t reg, uint8_t* data, size_t len) override {
        if (len == 0) return ESP_OK;
        
        spi_transaction_t t = {};
        t.addr = reg | 0x80;           // SPI Read: MSB 1 (핵심!)
        t.length = 8 * len;            // 읽을 전체 비트 수
        t.rxlength = 8 * len;          // 받을 전체 비트 수
        t.rx_buffer = data;            // 데이터를 저장할 버퍼 주소
        return spi_device_polling_transmit(_handle, &t);
    }
};


/**
 * 3. I2C 통신 구현체
 */
class I2CBus : public BusInterface {
private:
    i2c_master_dev_handle_t _handle;
    static constexpr uint32_t I2C_TIMEOUT_MS = 2;

public:
    I2CBus(i2c_master_dev_handle_t handle) : _handle(handle) {}
    ~I2CBus() override {
        if (_handle) i2c_master_bus_rm_device(_handle);
    }
    
    Interface::BusType get_type() const override { return Interface::BusType::I2C; }

    esp_err_t write(uint8_t reg, uint8_t data) override {
        uint8_t write_buf[2] = {reg, data};
        return i2c_master_transmit(_handle, write_buf, sizeof(write_buf), pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    }

    esp_err_t read(uint8_t reg, uint8_t* data, size_t len) override {
        // I2C는 주소 쓰기 후 데이터 읽기 (Transmit + Receive)
        return i2c_master_transmit_receive(_handle, &reg, 1, data, len, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    }
};


} // namespace Driver 끝
