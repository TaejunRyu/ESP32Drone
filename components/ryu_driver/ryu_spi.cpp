#include "ryu_spi.h"

namespace Driver {

namespace SPI {    
// static inline bmp388_calib_t bmp388_calib = {};

// static inline esp_err_t spi_transfer(
//         spi_device_handle_t dev,
//         const uint8_t *tx_data,
//         size_t tx_len,
//         uint8_t *rx_data,
//         size_t rx_len
// ) {
//     if (dev == nullptr) {
//         return ESP_ERR_INVALID_ARG;
//     }
//     const size_t total_len = tx_len + rx_len;
//     if (total_len == 0 || total_len > ICM20948_SPI_MAX_TRANSFER) {
//         return ESP_ERR_INVALID_SIZE;
//     }

//     uint8_t tx_buffer[ICM20948_SPI_MAX_TRANSFER] = {0};
//     std::memcpy(tx_buffer, tx_data, tx_len);
//     uint8_t rx_buffer[ICM20948_SPI_MAX_TRANSFER] = {0};
//     spi_transaction_t trans = {};
//     trans.length = total_len * 8;
//     trans.tx_buffer = tx_buffer;
//     trans.rx_buffer = rx_buffer;

//     esp_err_t ret = spi_device_transmit(dev, &trans);
//     if (ret != ESP_OK) {
//         return ret;
//     }

//     if (rx_len > 0 && rx_data != nullptr) {
//         std::memcpy(rx_data, rx_buffer + tx_len, rx_len);
//     }
//     return ESP_OK;
// }

// static inline esp_err_t write_register(
//         spi_device_handle_t dev,
//         uint8_t reg,
//         uint8_t value
// ) {
//     uint8_t cmd[2] = { static_cast<uint8_t>(reg & 0x7F), value };
//     spi_transaction_t trans = {};
//     trans.length = 16;
//     trans.tx_buffer = cmd;
//     return spi_device_transmit(dev, &trans);
// }

// static inline esp_err_t read_registers(
//         spi_device_handle_t dev,
//         uint8_t reg,
//         uint8_t *out_buf,
//         size_t len
// ) {
//     if (out_buf == nullptr || len == 0) {
//         return ESP_ERR_INVALID_ARG;
//     }
//     uint8_t command = static_cast<uint8_t>(reg | 0x80);
//     return spi_transfer(dev, &command, 1, out_buf, len);
// }

// static inline esp_err_t icm20948_spi_select_bank(spi_device_handle_t dev, uint8_t bank) {
//     return write_register(dev, 0x7F, static_cast<uint8_t>(bank << 4));
// }

// static inline esp_err_t initialize_spi_bus() {
//     spi_bus_config_t buscfg = {};
//     buscfg.miso_io_num = ICM20948_SPI_MISO;
//     buscfg.mosi_io_num = ICM20948_SPI_MOSI;
//     buscfg.sclk_io_num = ICM20948_SPI_SCLK;
//     buscfg.quadwp_io_num = -1;
//     buscfg.quadhd_io_num = -1;
//     buscfg.max_transfer_sz = ICM20948_SPI_MAX_TRANSFER;

//     esp_err_t ret = spi_bus_initialize(ICM20948_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
//     if (ret == ESP_ERR_INVALID_STATE) {
//         // 이미 초기화된 경우에도 정상 동작으로 간주
//         ret = ESP_OK;
//     }
//     if (ret == ESP_OK) {
//         ESP_LOGI("SPI", "SPI bus initialized: SCLK=%d MOSI=%d MISO=%d", ICM20948_SPI_SCLK, ICM20948_SPI_MOSI, ICM20948_SPI_MISO);
//     } else {
//         ESP_LOGE("SPI", "SPI bus initialize failed: %s", esp_err_to_name(ret));
//     }
//     return ret;
// }

// static inline esp_err_t add_icm20948_device(int index) {
//     if (index < 0 || index > 1) {
//         return ESP_ERR_INVALID_ARG;
//     }
//     gpio_num_t cs_pin = (index == 0) ? ICM20948_SPI_CS1 : ICM20948_SPI_CS2;

//     spi_device_interface_config_t devcfg = {};
//     devcfg.clock_speed_hz = ICM20948_SPI_CLOCK_HZ;
//     devcfg.mode = 0;
//     devcfg.spics_io_num = cs_pin;
//     devcfg.queue_size = 1;
//     devcfg.flags = 0;

//     return spi_bus_add_device(ICM20948_SPI_HOST, &devcfg, &icm20948_spi_handle[index]);
// }

// static inline esp_err_t initialize_icm20948_spi(int index) {
//     if (index < 0 || index > 1) {
//         return ESP_ERR_INVALID_ARG;
//     }

//     esp_err_t ret = initialize_spi_bus();
//     if (ret != ESP_OK) {
//         return ret;
//     }

//     ret = add_icm20948_device(index);
//     if (ret != ESP_OK) {
//         ESP_LOGE("SPI", "ICM20948 SPI device add failed for index %d: %s", index, esp_err_to_name(ret));
//         return ret;
//     }

//     spi_device_handle_t handle = icm20948_spi_handle[index];
//     ret = icm20948_spi_select_bank(handle, 0);
//     if (ret != ESP_OK) {
//         return ret;
//     }

//     ret = write_register(handle, ICM20948_B0_PWR_MGMT_1, 0x80);
//     if (ret != ESP_OK) {
//         return ret;
//     }
//     vTaskDelay(pdMS_TO_TICKS(100));

//     ret = write_register(handle, ICM20948_B0_PWR_MGMT_1, 0x01);
//     if (ret != ESP_OK) {
//         return ret;
//     }
//     vTaskDelay(pdMS_TO_TICKS(100));

//     ret = icm20948_spi_select_bank(handle, 2);
//     if (ret != ESP_OK) {
//         return ret;
//     }
//     vTaskDelay(pdMS_TO_TICKS(10));

//     ret = write_register(handle, ICM20948_B2_GYRO_CONFIG_1, 0x1D);
//     if (ret != ESP_OK) {
//         return ret;
//     }

//     ret = write_register(handle, ICM20948_B2_ACCEL_CONFIG, 0x1D);
//     if (ret != ESP_OK) {
//         return ret;
//     }

//     ret = icm20948_spi_select_bank(handle, 0);
//     if (ret != ESP_OK) {
//         return ret;
//     }
//     vTaskDelay(pdMS_TO_TICKS(10));

//     ESP_LOGI("SPI", "ICM20948 #%d SPI initialized", index);
//     return ESP_OK;
// }

// static inline esp_err_t enable_icm20948_aux_i2c_master(spi_device_handle_t handle) {
//     if (handle == nullptr) {
//         return ESP_ERR_INVALID_ARG;
//     }

//     esp_err_t ret = icm20948_spi_select_bank(handle, 0);
//     if (ret != ESP_OK) {
//         return ret;
//     }

//     // Enable I2C_MST_EN (bit 5 in USER_CTRL register)
//     ret = write_register(handle, ICM20948_B0_USER_CTRL, 0x20);
//     if (ret != ESP_OK) {
//         return ret;
//     }

//     // Set I2C Master Clock (0x24 register, bits 3:0 = clock divider)
//     // Default: 0x0D for ~400kHz
//     ret = write_register(handle, ICM20948_B0_I2C_MST_CTRL, 0x0D);
//     if (ret != ESP_OK) {
//         return ret;
//     }

//     ESP_LOGI("SPI", "ICM20948 Auxiliary I2C Master enabled");
//     return ESP_OK;
// }

// static inline esp_err_t setup_icm20948_ak09916_slave(spi_device_handle_t handle) {
//     if (handle == nullptr) {
//         return ESP_ERR_INVALID_ARG;
//     }

//     esp_err_t ret = icm20948_spi_select_bank(handle, 0);
//     if (ret != ESP_OK) {
//         return ret;
//     }

//     // Set Slave 0 Address (AK09916 = 0x0C, read bit set)
//     ret = write_register(handle, ICM20948_B0_I2C_SLV0_ADDR, (AK09916_ADDR | 0x80));
//     if (ret != ESP_OK) {
//         return ret;
//     }

//     // Set Slave 0 Register to read (AK09916_HXL = 0x11, data start)
//     ret = write_register(handle, ICM20948_B0_I2C_SLV0_REG, AK09916_HXL);
//     if (ret != ESP_OK) {
//         return ret;
//     }

//     // Enable Slave 0 and set read length (6 bytes for magnetometer data)
//     // Bit 7 = enabled, Bits 3:0 = length (6 bytes)
//     ret = write_register(handle, ICM20948_B0_I2C_SLV0_CTRL, 0x86);
//     if (ret != ESP_OK) {
//         return ret;
//     }

//     ESP_LOGI("SPI", "ICM20948 AK09916 Slave configured");
//     return ESP_OK;
// }

// static inline esp_err_t initialize_icm20948_spi_with_ak09916(int index) {
//     if (index < 0 || index > 1) {
//         return ESP_ERR_INVALID_ARG;
//     }

//     // First, initialize ICM20948 normally
//     esp_err_t ret = initialize_icm20948_spi(index);
//     if (ret != ESP_OK) {
//         return ret;
//     }

//     spi_device_handle_t handle = icm20948_spi_handle[index];

//     // Enable Auxiliary I2C Master mode
//     ret = enable_icm20948_aux_i2c_master(handle);
//     if (ret != ESP_OK) {
//         ESP_LOGE("SPI", "Failed to enable aux I2C master: %s", esp_err_to_name(ret));
//         return ret;
//     }

//     // Configure AK09916 as I2C Slave
//     ret = setup_icm20948_ak09916_slave(handle);
//     if (ret != ESP_OK) {
//         ESP_LOGE("SPI", "Failed to setup AK09916 slave: %s", esp_err_to_name(ret));
//         return ret;
//     }

//     vTaskDelay(pdMS_TO_TICKS(50));

//     ESP_LOGI("SPI", "ICM20948 #%d SPI with AK09916 Aux I2C initialized", index);
//     return ESP_OK;
// }

// static inline esp_err_t add_bmp388_device() {
//     spi_device_interface_config_t devcfg = {};
//     devcfg.clock_speed_hz = ICM20948_SPI_CLOCK_HZ;
//     devcfg.mode = 0;
//     devcfg.spics_io_num = BMP388_SPI_CS;
//     devcfg.queue_size = 1;
//     devcfg.flags = 0;

//     return spi_bus_add_device(ICM20948_SPI_HOST, &devcfg, &bmp388_spi_handle);
// }

// static inline void bmp388_precompute_coeffs(bmp388_calib_t &c) {
//     c._p1  = ((float)c.p1 - 16384.0f) / 1048576.0f;
//     c._p2  = ((float)c.p2 - 16384.0f) / 536870912.0f;
//     c._p3  = (float)c.p3 / 4294967296.0f;
//     c._p4  = (float)c.p4 / 137438953472.0f;
//     c._p5  = (float)c.p5 * 8.0f;
//     c._p6  = (float)c.p6 / 64.0f;
//     c._p7  = (float)c.p7 / 256.0f;
//     c._p8  = (float)c.p8 / 32768.0f;
//     c._p9  = (float)c.p9 / 281474976710656.0f;
//     c._p10 = (float)c.p10 / 281474976710656.0f;
//     c._p11 = (float)c.p11 / 36893488147419103232.0f;
// }

// static inline esp_err_t read_bmp388_calib(spi_device_handle_t handle, bmp388_calib_t &c) {
//     if (handle == nullptr) {
//         return ESP_ERR_INVALID_ARG;
//     }
//     uint8_t buf[21] = {0};
//     esp_err_t ret = read_registers(handle, BMP388_REG_CALIB, buf, sizeof(buf));
//     if (ret != ESP_OK) {
//         return ret;
//     }

//     c.t1 = (uint16_t)((uint16_t)buf[1] << 8 | buf[0]);
//     c.t2 = (uint16_t)((uint16_t)buf[3] << 8 | buf[2]);
//     c.t3 = (int8_t)buf[4];
//     c.p1 = (int16_t)((int16_t)buf[6] << 8 | buf[5]);
//     c.p2 = (int16_t)((int16_t)buf[8] << 8 | buf[7]);
//     c.p3 = (int8_t)buf[9];
//     c.p4 = (int8_t)buf[10];
//     c.p5 = (uint16_t)((uint16_t)buf[12] << 8 | buf[11]);
//     c.p6 = (uint16_t)((uint16_t)buf[14] << 8 | buf[13]);
//     c.p7 = (int8_t)buf[15];
//     c.p8 = (int8_t)buf[16];
//     c.p9 = (int16_t)((int16_t)buf[18] << 8 | buf[17]);
//     c.p10 = (int8_t)buf[19];
//     c.p11 = (int8_t)buf[20];

//     bmp388_precompute_coeffs(c);
//     return ESP_OK;
// }

// static inline float bmp388_compute_pressure(const bmp388_calib_t &c, uint32_t adc_p, uint32_t adc_t) {
//     float uncomp_t = (float)adc_t;
//     float uncomp_p = (float)adc_p;

//     float partial_t1 = uncomp_t - (float)c.t1 * 256.0f;
//     float partial_t2 = partial_t1 * (float)c.t2;
//     float t_lin = (partial_t2 / 1073741824.0f) + ((partial_t1 * partial_t1) * (float)c.t3 / 281474976710656.0f);

//     float s1 = c._p6 * t_lin;
//     float s2 = c._p7 * (t_lin * t_lin);
//     float s3 = c._p8 * (t_lin * t_lin * t_lin);
//     float partial_out1 = c._p5 + s1 + s2 + s3;

//     s1 = c._p2 * t_lin;
//     s2 = c._p3 * (t_lin * t_lin);
//     s3 = c._p4 * (t_lin * t_lin * t_lin);
//     float partial_out2 = uncomp_p * (c._p1 + s1 + s2 + s3);

//     float d1 = uncomp_p * uncomp_p;
//     float d2 = c._p9 + c._p10 * t_lin;
//     float d3 = d1 * d2;
//     float d4 = d3 + (uncomp_p * uncomp_p * uncomp_p) * c._p11;

//     return (partial_out1 + partial_out2 + d4) * 0.01f;
// }

// static inline esp_err_t add_bmp388_device_and_calib() {
//     esp_err_t ret = add_bmp388_device();
//     if (ret != ESP_OK) {
//         return ret;
//     }
//     return read_bmp388_calib(bmp388_spi_handle, bmp388_calib);
// }

// static inline esp_err_t initialize_bmp388_spi() {
//     esp_err_t ret = initialize_spi_bus();
//     if (ret != ESP_OK) {
//         return ret;
//     }
//     ret = add_bmp388_device();
//     if (ret != ESP_OK) {
//         ESP_LOGE("SPI", "BMP388 SPI device add failed: %s", esp_err_to_name(ret));
//         return ret;
//     }

//     ret = write_register(bmp388_spi_handle, BMP388_CMD_SOFT_RESET, 0xB6);
//     if (ret != ESP_OK) {
//         return ret;
//     }
//     vTaskDelay(pdMS_TO_TICKS(100));

//     ret = write_register(bmp388_spi_handle, BMP388_REG_PWR_CTRL, 0x00);
//     if (ret != ESP_OK) {
//         return ret;
//     }

//     ret = write_register(bmp388_spi_handle, BMP388_REG_OSR, (0x03 << 0) | (0x01 << 3));
//     if (ret != ESP_OK) {
//         return ret;
//     }

//     ret = write_register(bmp388_spi_handle, BMP388_REG_IIR, 0x02 << 1);
//     if (ret != ESP_OK) {
//         return ret;
//     }

//     ret = write_register(bmp388_spi_handle, BMP388_REG_ODR, 0x02);
//     if (ret != ESP_OK) {
//         return ret;
//     }

//     ret = write_register(bmp388_spi_handle, BMP388_REG_PWR_CTRL, 0x13);
//     if (ret != ESP_OK) {
//         return ret;
//     }
//     vTaskDelay(pdMS_TO_TICKS(50));

//     ret = write_register(bmp388_spi_handle, BMP388_REG_PWR_CTRL, 0x33);
//     if (ret != ESP_OK) {
//         return ret;
//     }
//     vTaskDelay(pdMS_TO_TICKS(50));

//     ret = read_bmp388_calib(bmp388_spi_handle, bmp388_calib);
//     if (ret != ESP_OK) {
//         ESP_LOGE("SPI", "BMP388 calibration read failed: %s", esp_err_to_name(ret));
//         return ret;
//     }

//     ESP_LOGI("SPI", "BMP388 SPI initialized");
//     return ESP_OK;
// }

// static inline std::tuple<esp_err_t, uint32_t, uint32_t> read_bmp388_raw(spi_device_handle_t handle) {
//     if (handle == nullptr) {
//         return {ESP_ERR_INVALID_ARG, 0, 0};
//     }

//     uint8_t buf[6] = {0};
//     esp_err_t ret = read_registers(handle, BMP388_REG_DATA, buf, sizeof(buf));
//     if (ret != ESP_OK) {
//         return {ret, 0, 0};
//     }

//     uint32_t adc_p = ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | buf[0];
//     uint32_t adc_t = ((uint32_t)buf[5] << 16) | ((uint32_t)buf[4] << 8) | buf[3];
//     return {ESP_OK, adc_p, adc_t};
// }

// static inline std::tuple<esp_err_t, float> read_bmp388_pressure(spi_device_handle_t handle) {
//     if (handle == nullptr) {
//         return {ESP_ERR_INVALID_STATE, 0.0f};
//     }

//     uint32_t adc_p, adc_t;
//     esp_err_t ret;
//     std::tie(ret, adc_p, adc_t) = read_bmp388_raw(handle);
//     if (ret != ESP_OK) {
//         return {ret, 0.0f};
//     }
//     float pressure = bmp388_compute_pressure(bmp388_calib, adc_p, adc_t);
//     return {ESP_OK, pressure};
// }

// static inline std::tuple<esp_err_t, std::array<float,3>> read_ak09916_with_offset(i2c_master_dev_handle_t handle) {
//     return AK09916::read_with_offset(handle);
// }

// static inline esp_err_t enable_icm20948_mag_bypass_spi(spi_device_handle_t handle) {
//     if (handle == nullptr) {
//         return ESP_ERR_INVALID_ARG;
//     }

//     esp_err_t ret = icm20948_spi_select_bank(handle, 0);
//     if (ret != ESP_OK) {
//         return ret;
//     }

//     // Disable I2C master to enable bypass mode
//     ret = write_register(handle, ICM20948_B0_USER_CTRL, 0x00);
//     if (ret != ESP_OK) {
//         return ret;
//     }

//     // Enable bypass mode
//     ret = write_register(handle, ICM20948_B0_INT_PIN_CFG, 0x02);
//     if (ret == ESP_OK) {
//         ESP_LOGI("SPI", "ICM20948 mag bypass SPI enabled");
//     }
//     return ret;
// }



// static inline std::tuple<esp_err_t, std::array<float,3>, std::array<float,3>, std::array<float,3>> 
// read_icm20948_with_ak09916_spi(spi_device_handle_t handle) {
//     if (handle == nullptr) {
//         return {ESP_ERR_INVALID_ARG, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
//     }

//     esp_err_t ret = icm20948_spi_select_bank(handle, 0);
//     if (ret != ESP_OK) {
//         return {ret, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
//     }

//     // Read Accel + Gyro (12 bytes) + External Sensor Data (Mag from AK09916, 6 bytes)
//     uint8_t buf[18] = {0};
//     ret = read_registers(handle, ICM20948_B0_ACCEL_XOUT_H, buf, sizeof(buf));
//     if (ret != ESP_OK) {
//         return {ret, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
//     }

//     // Parse Accelerometer (bytes 0-5)
//     std::array<float,3> acc = {};
//     acc[X]  = static_cast<int16_t>((buf[0] << 8) | buf[1]) / 4096.0f;
//     acc[Y]  = static_cast<int16_t>((buf[2] << 8) | buf[3]) / 4096.0f;
//     acc[Z]  = static_cast<int16_t>((buf[4] << 8) | buf[5]) / 4096.0f;

//     // Parse Gyroscope (bytes 6-11)
//     std::array<float,3> gyro = {};
//     gyro[X] = static_cast<int16_t>((buf[6] << 8) | buf[7]) / 32.8f;
//     gyro[Y] = static_cast<int16_t>((buf[8] << 8) | buf[9]) / 32.8f;
//     gyro[Z] = static_cast<int16_t>((buf[10] << 8) | buf[11]) / 32.8f;

//     // Parse Magnetometer (bytes 12-17) from external sensor data (AK09916)
//     // AK09916 data format: HXL(0x11), HXH, HYL, HYH, HZL, HZH
//     std::array<float,3> mag = {};
//     mag[X] = static_cast<int16_t>((buf[13] << 8) | buf[12]) * 0.15f;  // µT
//     mag[Y] = static_cast<int16_t>((buf[15] << 8) | buf[14]) * 0.15f;
//     mag[Z] = static_cast<int16_t>((buf[17] << 8) | buf[16]) * 0.15f;

//     return {ESP_OK, acc, gyro, mag};
// }

// static inline std::tuple<esp_err_t, std::array<float,3>, std::array<float,3>> 
// read_icm20948_raw(spi_device_handle_t handle) {
//     if (handle == nullptr) {
//         return {ESP_ERR_INVALID_ARG, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
//     }
//     esp_err_t ret = icm20948_spi_select_bank(handle, 0);
//     if (ret != ESP_OK) {
//         return {ret, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
//     }

//     uint8_t buf[12] = {0};
//     ret = read_registers(handle, ICM20948_B0_ACCEL_XOUT_H, buf, sizeof(buf));
//     if (ret != ESP_OK) {
//         return {ret, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
//     }

//     std::array<float,3> acc = {};
//     std::array<float,3> gyro = {};
//     acc[X]  = static_cast<int16_t>((buf[0] << 8) | buf[1]) / 4096.0f;
//     acc[Y]  = static_cast<int16_t>((buf[2] << 8) | buf[3]) / 4096.0f;
//     acc[Z]  = static_cast<int16_t>((buf[4] << 8) | buf[5]) / 4096.0f;
//     gyro[X] = static_cast<int16_t>((buf[6] << 8) | buf[7]) / 32.8f;
//     gyro[Y] = static_cast<int16_t>((buf[8] << 8) | buf[9]) / 32.8f;
//     gyro[Z] = static_cast<int16_t>((buf[10] << 8) | buf[11]) / 32.8f;

//     return {ESP_OK, acc, gyro};
// }

// static inline std::tuple<esp_err_t, std::array<float,3>, std::array<float,3>> read_icm20948_with_offset(
//         spi_device_handle_t handle,
//         const std::array<float,3>& offset_acc,
//         const std::array<float,3>& offset_gyro
// ) {
//     auto [ret, acc, gyro] = read_icm20948_raw(handle);
//     if (ret != ESP_OK) {
//         return {ret, acc, gyro};
//     }

//     acc[X]  -= offset_acc[X];
//     acc[Y]  -= offset_acc[Y];
//     acc[Z]  -= offset_acc[Z];
//     gyro[X] -= offset_gyro[X];
//     gyro[Y] -= offset_gyro[Y];
//     gyro[Z] -= offset_gyro[Z];

//     acc[Y]  *= -1.0f;
//     gyro[X] *= -1.0f;
//     gyro[Z] *= -1.0f;

//     return {ESP_OK, acc, gyro};
// }

// static inline esp_err_t initialize_all_icm20948_spi() {
//     esp_err_t ret = initialize_spi_bus();
//     if (ret != ESP_OK) {
//         return ret;
//     }
//     for (int i = 0; i < 2; ++i) {
//         ret = initialize_icm20948_spi(i);
//         if (ret != ESP_OK) {
//             ESP_LOGE("SPI", "ICM20948 SPI init failed for index %d: %s", i, esp_err_to_name(ret));
//             return ret;
//         }
//     }
//     return ESP_OK;
// }

} // namespace SPI
} // namespace Driver
// 74HC138: "8개 중 하나만 골라!" (선택)
// 74HC595: "핀 3개로 여러 개를 내 맘대로 켜고 유지해!" (직렬 확장)
// 74HC573(래치): "지금 이 데이터를 그대로 기억해!" (데이터 보존)
//
//상황 (사용 목적)                              추천 부품   특징 (한 줄 요약)
//8개 중 하나만 딱 골라 쓰고 싶을 때            74HC138     가장 기본적이고 빠른 '선택기' (디코더)
//여러 개 센서 값을 하나씩 읽고 싶을 때         CD4051      아날로그/디지털 신호를 양방향으로 통과시키는 '통로'
//일반 GPIO 핀처럼 입출력을 늘리고 싶을 때      MCP23017I2C 통신으로 핀 16개를 진짜 내 핀처럼 추가 (강력 추천)
//LED나 릴레이 여러 개를 동시에 켜고 싶을 때    74HC595     적은 핀으로 많은 출력을 제어하는 '가성비' 끝판왕
//모터 속도(PWM)를 정밀하게 제어하고 싶을 때    PCA9685     CPU 부하 없이 16채널 PWM 신호를 스스로 유지

//  Usage Examples for SPI Sensor Functions
//
//  This section provides examples of how to use the SPI functions defined above.
//  These examples assume you have included this header and are working in an ESP32 environment.
//
//  1. Initialize SPI Bus and ICM20948 Sensors (Bypass Mode - Separate I2C):
//
//     esp_err_t ret = SPI::initialize_all_icm20948_spi();
//     if (ret != ESP_OK) {
//         ESP_LOGE("MAIN", "Failed to initialize ICM20948 SPI: %s", esp_err_to_name(ret));
//         return ret;
//     }
//
//     // Enable magnetometer bypass for ICM20948 #0 (to use external I2C for AK09916)
//     ret = SPI::enable_icm20948_mag_bypass_spi(SPI::icm20948_spi_handle[0]);
//     if (ret != ESP_OK) {
//         ESP_LOGE("MAIN", "Failed to enable mag bypass: %s", esp_err_to_name(ret));
//     }
//
//  2. RECOMMENDED: Initialize ICM20948 with Auxiliary I2C Master (Read AK09916 Together):
//
//     // Initialize ICM20948 with integrated AK09916 support via Auxiliary I2C Master
//     esp_err_t ret = SPI::initialize_icm20948_spi_with_ak09916(0);
//     if (ret != ESP_OK) {
//         ESP_LOGE("MAIN", "Failed to initialize with AK09916: %s", esp_err_to_name(ret));
//         return ret;
//     }
//
//     // Now you can read Accel + Gyro + Magnetometer in ONE call
//     // ICM20948 automatically reads from AK09916 via its internal I2C master
//     auto [ret, acc, gyro, mag] = SPI::read_icm20948_with_ak09916_spi(SPI::icm20948_spi_handle[0]);
//     if (ret == ESP_OK) {
//         ESP_LOGI("SENSOR", "Accel: X=%.2f, Y=%.2f, Z=%.2f g", acc[0], acc[1], acc[2]);
//         ESP_LOGI("SENSOR", "Gyro:  X=%.2f, Y=%.2f, Z=%.2f dps", gyro[0], gyro[1], gyro[2]);
//         ESP_LOGI("SENSOR", "Mag:   X=%.2f, Y=%.2f, Z=%.2f µT", mag[0], mag[1], mag[2]);
//     } else {
//         ESP_LOGE("SENSOR", "Failed to read sensors: %s", esp_err_to_name(ret));
//     }
//
//  3. Auxiliary I2C Master Overview:
//
//     ICM20948 has an internal I2C master that can read from external I2C slaves.
//     Benefits:
//     - AK09916 data synchronized with IMU data (all in one read)
//     - Simplified MCU pin usage (no separate I2C pins for AK09916)
//     - Lower MCU I2C bus load
//     - Single SPI interface from MCU handles both IMU and Mag internally
//
//     How it works:
//     1. enable_icm20948_aux_i2c_master() activates ICM20948's internal I2C master
//     2. setup_icm20948_ak09916_slave() configures AK09916 as an I2C slave
//     3. ICM20948 automatically reads AK09916 data during each measurement cycle
//     4. read_icm20948_with_ak09916_spi() retrieves all sensor data at once
//        (Accel, Gyro, Magnetometer - 18 bytes total)
//
//  4. Read ICM20948 Raw Data with Offsets:
//
//     std::array<float, 3> acc_offset = {0.1f, 0.2f, 0.3f};
//     std::array<float, 3> gyro_offset = {0.01f, 0.02f, 0.03f};
//
//     auto [ret, acc, gyro] = SPI::read_icm20948_with_offset(SPI::icm20948_spi_handle[0], acc_offset, gyro_offset);
//     if (ret == ESP_OK) {
//         ESP_LOGI("SENSOR", "Accel: X=%.2f, Y=%.2f, Z=%.2f", acc[0], acc[1], acc[2]);
//         ESP_LOGI("SENSOR", "Gyro: X=%.2f, Y=%.2f, Z=%.2f", gyro[0], gyro[1], gyro[2]);
//     } else {
//         ESP_LOGE("SENSOR", "Failed to read ICM20948: %s", esp_err_to_name(ret));
//     }
//
//  5. Initialize and Read BMP388 Pressure Sensor:
//
//     esp_err_t ret = SPI::initialize_bmp388_spi();
//     if (ret != ESP_OK) {
//         ESP_LOGE("MAIN", "Failed to initialize BMP388 SPI: %s", esp_err_to_name(ret));
//         return ret;
//     }
//
//     auto [ret, pressure] = SPI::read_bmp388_pressure(SPI::bmp388_spi_handle);
//     if (ret == ESP_OK) {
//         ESP_LOGI("SENSOR", "Pressure: %.2f hPa", pressure);
//     } else {
//         ESP_LOGE("SENSOR", "Failed to read BMP388: %s", esp_err_to_name(ret));
//     }
//
//  Note: Approach #2 (Auxiliary I2C Master) is RECOMMENDED for synchronized multi-sensor reads.
//  This provides the best performance and reduces MCU overhead significantly.


//  Note: These examples are for illustrative purposes. In actual use, you should handle errors appropriately,
//  manage sensor calibration, and consider threading/synchronization for real-time applications.
