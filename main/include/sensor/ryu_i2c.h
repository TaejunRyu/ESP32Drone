#pragma once

#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include "ryu_config.h"
namespace I2C{
extern i2c_master_bus_handle_t initialize(i2c_port_num_t port, gpio_num_t port_sda, gpio_num_t port_scl);
extern void scan_bus(i2c_master_bus_handle_t handle);
extern i2c_master_bus_handle_t i2c_bus_hardware_clear(i2c_master_bus_handle_t old_handle);
}