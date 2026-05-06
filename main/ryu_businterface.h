#pragma once

#include <esp_err.h>
//#include <freertos/FreeRTOS.h> // pdMS_TO_TICKS 사용을 위해 필요
#include <driver/spi_master.h>
#include <driver/i2c_master.h>

namespace Driver {

// 1. 핸들(Handle) 준비 (하드웨어 연결)
//     1.1 I2C 핸들: i2c_master_dev_handle_t (어떤 주소의 장치인가?)
//     1.2 SPI 핸들: spi_device_handle_t (어떤 CS 핀을 쓰는 장치인가?)
// 2. 인터페이스 접목 (드라이버 장착)이제 이 핸들을 각 통신용 클래스(I2CBus 또는 SPIBus)에 담습니다. 
//     이 클래스들은 BusInterface라는 옷을 입고 있기 때문에, 밖에서 보기에는 똑같은 write/read 도구로 보입니다.
//     // 핸들(열쇠)을 인터페이스(도구)에 끼움
//     Driver::SPIBus mySpiBus(spi_handle); 
//     Driver::I2CBus myI2cBus(i2c_handle);
// 3. 일을 시키기 (센서 구동)센서 클래스(ICM20948)는 
//     자신이 가진 BusInterface* _bus 포인터가 실제로는 SPI 핸들을 쓰는지 I2C 핸들을 쓰는지 묻지 않고 그냥 일을 시킵니다.
//     // 센서에게 도구(인터페이스)를 건네줌
//     imu.set_bus(&mySpiBus);
//     // 일을 시킴 (내부적으로 핸들을 사용하여 하드웨어 제어)
//     imu.initialize(); 
//     imu.read_imu();

// 1. 버스(Host) 초기화 (고속도로 건설)
//      Driver::SPI::initialize() 안에서 처리됨
// 2. 개별 장치(Handle) 등록 (차량 등록)
//      spi_device_handle_t icm_handle;
//      spi_bus_add_device(SPI2_HOST, &dev_cfg, &icm_handle);
// 3. 인터페이스 클래스에 Handle 전달 (운전사에게 열쇠 전달)
//      Driver::SPIBus imu_spi_bus(icm_handle); 
// 4. 센서에 인터페이스 주입
//      imu.set_bus(&imu_spi_bus);


// class ICM20948 {
// public:
//     // ★ 핵심: 어떤 버스(I2C 또는 SPI)를 사용할지 주입하는 함수
//     void set_bus(Driver::BusInterface* bus) { _bus = bus; }
// private:
//     // 드라이버 핸들 대신 인터페이스 포인터 사용
//     Driver::BusInterface* _bus = nullptr; 
// };



//  함수내부에서 read,write부분을 interface에게 처리하도록 한다.
// std::tuple<esp_err_t, std::array<float, 3>, std::array<float, 3>> ICM20948::read_raw_data()
// {
//     // 초기화 및 버스 설정 확인
//     if (!_initialized || _bus == nullptr) { 
//         return {ESP_FAIL, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
//     }

//     uint8_t d[12]; // Accel(6바이트) + Gyro(6바이트) 전송용 버퍼
    
//     // 데이터 읽기 전 뱅크 0 확인
//     icm20948_select_bank(0);

//     // 인터페이스를 통한 연속 읽기 (0x2D부터 12바이트)
//     // I2C/SPI 구현체 내부에서 각각의 프로토콜에 맞게 데이터를 채워줍니다.
//     esp_err_t ret = _bus->read(B0_ACCEL_XOUT_H, d, 12);
    
//     if (ret != ESP_OK) {
//         return {ret, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
//     }

//     std::array<float, 3> acc, gyro;

//     // 가속도 데이터 변환 (±8g 설정 기준: 4096 LSB/g)
//     acc[0] = (int16_t)((d[0] << 8) | d[1]) / 4096.0f;
//     acc[1] = (int16_t)((d[2] << 8) | d[3]) / 4096.0f;
//     acc[2] = (int16_t)((d[4] << 8) | d[5]) / 4096.0f;

//     // 자이로 데이터 변환 (±1000dps 설정 기준: 32.8 LSB/dps)
//     gyro[0] = (int16_t)((d[6] << 8) | d[7]) / 32.8f;
//     gyro[1] = (int16_t)((d[8] << 8) | d[9]) / 32.8f;
//     gyro[2] = (int16_t)((d[10] << 8) | d[11]) / 32.8f;
    
//     return {ESP_OK, acc, gyro};
// }



// 실제 사용시점에 이렇게 사용한다.
// void app_main() {
//     //1. SPI 하드웨어 초기화 및 핸들 획득 (앞서 만든 Driver::SPI 사용)
//     spi_device_handle_t my_spi_handle = ... (생략) ...

//     // 2. SPI 통신 도구 생성
//     static Driver::SPIBus spi_comm(my_spi_handle);

//     // 3. 센서 준비 및 통신 도구 연결
//     auto& imu = Sensor::ICM20948::get_instance();
//     imu.set_bus(&spi_comm); // 이 줄 하나로 SPI 사용 결정!
//}


/**
 * 1. 통신 인터페이스 (부모)
 */
class BusInterface {
public:
    virtual ~BusInterface() = default;
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
    static constexpr uint32_t I2C_TIMEOUT_MS = 50;

public:
    I2CBus(i2c_master_dev_handle_t handle) : _handle(handle) {}

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
