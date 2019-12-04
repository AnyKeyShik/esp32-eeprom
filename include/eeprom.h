#pragma once

#include <stdio.h>
#include "driver/i2c.h"

// Definitions for i2c
#define I2C_MASTER_SCL_IO   18                  // GPIO for SCL
#define I2C_MASTER_SDA_IO   19                  // GPIO for SDA
#define I2C_MASTER_NUM  I2C_NUM_1               // I2C port
#define I2C_MASTER_TX_BUF_DISABLE   0           // I2C master doesn't need buffer 
#define I2C_MASTER_RX_BUF_DISABLE   0           // I2C master doesn't need buffer
#define I2C_MASTER_FREQ_HZ  100000              // I2C master clock frequency
#define PULLUP_SCL_GPIO  GPIO_PULLUP_ENABLE     // Enable pull-up on SCL GPIO
#define PULLUP_SDA_GPIO  GPIO_PULLUP_ENABLE     // Enable pull-up on SDA GPIO

#define ACK_CHECK_EN 0x1                        // I2C master will check ack from EEPROM
#define ACK_CHECK_DIS 0x0                       // I2C master will not check ack from EEPROM
#define ACK_VAL 0x0                             // I2C ack value
#define NACK_VAL 0x1                            // I2C nack value

#define EEPROM_WRITE_ADDR   0x00                // Write bit for EEPROM
#define EEPROM_READ_ADDR    0x01                // Read bit for EEPROM

#define EEPROM_PAGE_SIZE	32                  // Page size of EEPROM

esp_err_t init_i2c_master();

esp_err_t eeprom_write_byte(uint8_t, uint16_t, uint8_t);
esp_err_t eeprom_write(uint8_t, uint16_t, uint8_t*, size_t);

esp_err_t eeprom_read_byte(uint8_t, uint16_t, uint8_t *);
esp_err_t eeprom_read(uint8_t, uint16_t, uint8_t *, size_t);
