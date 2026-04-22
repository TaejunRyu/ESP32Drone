#pragma once

#include <cstring>
#include <cmath>
#include <array>
#include <tuple>

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include "ryu_ak09916.h"
#include "ryu_config.h"

namespace SPI {

spi_host_device_t ICM20948_SPI_HOST;
inline constexpr gpio_num_t ICM20948_SPI_SCLK = VSPI_SCLK;
inline constexpr gpio_num_t ICM20948_SPI_MISO = VSPI_MISO;
inline constexpr gpio_num_t ICM20948_SPI_MOSI = VSPI_MOSI;
inline constexpr gpio_num_t ICM20948_SPI_CS1  = VSPI_CS1;  // ICM20948 #1
inline constexpr gpio_num_t ICM20948_SPI_CS2  = VSPI_CS2;  // ICM20948 #2
inline constexpr uint32_t ICM20948_SPI_CLOCK_HZ = 4'000'000; // 4 MHz
inline constexpr size_t   ICM20948_SPI_MAX_TRANSFER = 256;

inline constexpr uint8_t ICM20948_REG_BANK_SEL = 0x7F;
inline constexpr uint8_t ICM20948_B0_ACCEL_XOUT_H = 0x2D;
inline constexpr uint8_t ICM20948_B0_GYRO_XOUT_H  = 0x33;
inline constexpr uint8_t ICM20948_B0_PWR_MGMT_1   = 0x06;
inline constexpr uint8_t ICM20948_B0_INT_PIN_CFG  = 0x0F;
inline constexpr uint8_t ICM20948_B0_USER_CTRL    = 0x03;

inline constexpr uint8_t ICM20948_B0_I2C_MST_CTRL = 0x24;  // Auxiliary I2C Master Control
inline constexpr uint8_t ICM20948_B0_I2C_SLV0_ADDR= 0x25;  // Slave 0 Address
inline constexpr uint8_t ICM20948_B0_I2C_SLV0_REG = 0x26;  // Slave 0 Register
inline constexpr uint8_t ICM20948_B0_I2C_SLV0_CTRL= 0x27;  // Slave 0 Control

inline constexpr uint8_t ICM20948_B0_EXT_SLV_SENS_DATA_00 = 0x3B;  // External Sensor Data (Mag start)
inline constexpr uint8_t ICM20948_B2_GYRO_CONFIG_1= 0x01;
inline constexpr uint8_t ICM20948_B2_ACCEL_CONFIG = 0x14;


inline constexpr uint8_t AK09916_ADDR            = 0x0C;   // AK09916 I2C Address
inline constexpr uint8_t AK09916_HXL             = 0x11;   // Status Register & Data Start
inline constexpr uint8_t AK09916_CNTL2           = 0x31;
inline constexpr uint8_t AK09916_CNTL3           = 0x32;

inline constexpr gpio_num_t BMP388_SPI_CS    = VSPI_CS3;  // BMP388 SPI CS
inline constexpr uint8_t BMP388_REG_ID       = 0x00;
inline constexpr uint8_t BMP388_REG_DATA     = 0x04;
inline constexpr uint8_t BMP388_REG_PWR_CTRL = 0x1B;
inline constexpr uint8_t BMP388_REG_CALIB    = 0x31;
inline constexpr uint8_t BMP388_REG_STATUS   = 0x03;
inline constexpr uint8_t BMP388_REG_OSR      = 0x1C;
inline constexpr uint8_t BMP388_REG_IIR      = 0x1F;
inline constexpr uint8_t BMP388_REG_ODR      = 0x1D;
inline constexpr uint8_t BMP388_CMD_SOFT_RESET = 0x7E;

inline spi_device_handle_t icm20948_spi_handle[2] = { nullptr, nullptr };
inline spi_device_handle_t bmp388_spi_handle = nullptr;

struct bmp388_calib_t {
    uint16_t t1;
    uint16_t t2;
    int8_t t3;
    int16_t p1;
    int16_t p2;
    int8_t p3;
    int8_t p4;
    uint16_t p5;
    uint16_t p6;
    int8_t p7;
    int8_t p8;
    int16_t p9;
    int8_t p10;
    int8_t p11;
    float _p1;
    float _p2;
    float _p3;
    float _p4;
    float _p5;
    float _p6;
    float _p7;
    float _p8;
    float _p9;
    float _p10;
    float _p11;
};

}
