# I2C driver for EEPROM on ESP32

Simple driver for work with EEPROM on ESP32 according to the I2C protocol

## Constants
* I2C_MASTER_SCL_IO

        GPIO for SCL


* I2C_MASTER_SDA_IO

        GPIO for SDA


* I2C_MASTER_NUM

        I2C port


* I2C_MASTER_TX_BUF_DISABLE

        TX buffer disable(0)/enable(1)


* I2C_MASTER_RX_BUF_DISABLE

        RX buffer disable(0)/enable(1)


* I2C_MASTER_FREQ_HZ

        Clock frequency


* PULLUP_SCL_GPIO

        Enable pull-up on SCL GPIO


* PULLUP_SDA_GPIO

        Enable pull-up on SDA GPIO


* ACK_CHECK_EN

        Check ACK from EEPROM


* ACK_CHECK_DIS
        
        Don't check ACK from EEPROM


* ACK_VAL

        ACK value


* NACK_VAL

        NACK value


* EEPROM_WRITE_ADDR

        Write bit for EEPROM


* EEPROM_READ_ADDR

        Read bit for EEPROM


* EEPROM_PAGE_SIZE

        Page size of EEPROM


## Provided functions
* esp_err_t init_i2c_master()

        I2C master initialization with PULLUP_`X`_GPIO pull-up on GPIOs I2C_MASTER_SDA_IO and I2C_MASTER_SCL_IO with I2C_MASTER_FREQ_HZ frequency on I2C_MODE_MASTER
    
        Return ESP_OK or error


* esp_err_t eeprom_write_byte(uint8_t deviceaddress, uint16_t eeaddress, uint8_t byte)

        Write one byte to EEPROM
        
        * deviceaddress - EEPROM address
        * eeaddress - address on EEPROM where byte will be write
        * byte - byte for write

        Write schema in I2C:
        ___________________________________________________________________________________________________________
        | start | slave_addr + wr_bit + ack | high_byte_aadr + ack | low_byte_addr + ack | data_byte + ack | stop |
        |-------|---------------------------|----------------------|---------------------|-----------------|------|

        Return ESP_OK or error


* esp_err_t eeprom_write(uint8_t deviceaddress, uint16_t eeaddress, uint8_t *data, size_t size)

        Write a few bytes to EEPROM

        * deviceaddress - EEPROM address
        * eeaddress - address on EEPROM where bytes will be write 
        * data - bytes for write
        * size - size of data

        Write schema in I2C:
        _______________________________________________________________________________________________________________________________________
        | start | slave_addr + wr_bit + ack | high_byte_aadr + ack | low_byte_addr + ack | data_byte_0 + ack | ... | data_byte_x + ack | stop |
        |-------|---------------------------|----------------------|---------------------|------------------|-----|--------------------|------|

        Return ESP_OK or error


* esp_err_t eeprom_read_byte(uint8_t deviceaddress, uint16_t eeaddress, uint8_t *byte)

        Read random byte from EEPROM

        * deviceaddress - EEPROM address
        * eeaddress - EEPROM address from which the byte will be read
        * byte - var into which byte will be read

        Read schema in I2C:
        __________________________________________________________________________________________________________________________________________________
        | start | slave_addr + wr_bit + ack | high_byte_addr + ack | low_byte_addr + ack | start | slave_addr + rd_bit + ack | read 1 byte + nack | stop |
        |-------|---------------------------|----------------------|---------------------|-------|---------------------------|--------------------|------|

        Return ESP_OK or error


* esp_err_t eeprom_read(uint8_t deviceaddress, uint16_t eeaddress, uint8_t *data, size_t size)

        Read byte sequential from EEPROM

        * deviceaddress - EEPROM address
        * eeaddress - EEPROM address from which the bytes will be read
        * data - var into which bytes will be read
        
        Read schema in I2C:

        ____________________________________________________________________________________________________________________________________________________________________________
        | start | slave_addr + wr_bit + ack | high_byte_addr + ack | low_byte_addr + ack | start | slave_addr + rd_bit + ack | read byte_1 + ack | ... | read byte_x + nack | stop |
        |-------|---------------------------|----------------------|---------------------|-------|---------------------------|-------------------|-----|--------------------|------|

        Return ESP_OK or error
