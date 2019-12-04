#include "eeprom.h"

/**
 * @brief I2C master initialization
 */
esp_err_t init_i2c_master() {
    int i2c_master_port = I2C_MASTER_NUM;
    esp_err_t ret;

    i2c_config_t config;
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = I2C_MASTER_SDA_IO;
    config.scl_io_num = I2C_MASTER_SCL_IO;
    config.sda_pullup_en = PULLUP_SDA_GPIO;
    config.scl_pullup_en = PULLUP_SCL_GPIO;
    config.master.clk_speed = I2C_MASTER_FREQ_HZ;
    
    ret = i2c_param_config(i2c_master_port, &config);
    if(ret != ESP_OK) {
        return ret;
    }

    ret = i2c_driver_install(i2c_master_port, config.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

    return ret;
}

/**
 * @brief Write one byte to EEPROM.
 *
 * ___________________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | high_byte_aadr + ack | low_byte_addr + ack | data_byte + ack | stop |
 * |-------|---------------------------|----------------------|---------------------|-----------------|------|
 *
 */
esp_err_t eeprom_write_byte(uint8_t deviceaddress, uint16_t eeaddress, uint8_t byte) {
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress << 1) | EEPROM_WRITE_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, eeaddress >> 8, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, eeaddress & 0xFF, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, byte, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

/**
 * @brief Write a few bytes to EEPROM.
 *
 * _______________________________________________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | high_byte_aadr + ack | low_byte_addr + ack | data_byte0 + ack | ... | data_byte127 + ack | stop |
 * |-------|---------------------------|----------------------|---------------------|------------------|-----|--------------------|------|
 *
 */
esp_err_t eeprom_write(uint8_t deviceaddress, uint16_t eeaddress, uint8_t *data, size_t size) {
    esp_err_t ret = ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress << 1) | EEPROM_WRITE_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, eeaddress >> 8, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, eeaddress & 0xFF, ACK_CHECK_EN);

    int bytes_remaining = size;
    int current_address = eeaddress;
    int first_write_size = ( (EEPROM_PAGE_SIZE - 1) - eeaddress % (EEPROM_PAGE_SIZE - 1) ) + 1;
    
    if (eeaddress % (EEPROM_PAGE_SIZE-1) == 0 && eeaddress != 0) {
        first_write_size = 1;
    }

    if (bytes_remaining <= first_write_size) {
        i2c_master_write(cmd, data, bytes_remaining, 1);
    } 
    else {
        i2c_master_write(cmd, data, first_write_size, 1);
        bytes_remaining -= first_write_size;
        current_address += first_write_size;
        i2c_master_stop(cmd);
        
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000/portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret != ESP_OK) { 
            return ret;
        }

        while (bytes_remaining > 0) {
            cmd = i2c_cmd_link_create();

            // 2ms delay period to allow EEPROM to write the page buffer to memory.
            vTaskDelay(20 / portTICK_PERIOD_MS);

            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (deviceaddress << 1) | EEPROM_WRITE_ADDR, ACK_CHECK_EN);
            i2c_master_write_byte(cmd, current_address >> 8, ACK_CHECK_EN);
            i2c_master_write_byte(cmd, current_address & 0xFF, ACK_CHECK_EN);

            if (bytes_remaining <= EEPROM_PAGE_SIZE) {
                i2c_master_write(cmd, data + (size - bytes_remaining), bytes_remaining, ACK_CHECK_EN);
                bytes_remaining = 0;
            } 
            else {
                i2c_master_write(cmd, data + (size - bytes_remaining), EEPROM_PAGE_SIZE, ACK_CHECK_EN);
                bytes_remaining -= EEPROM_PAGE_SIZE;
                current_address += EEPROM_PAGE_SIZE;
            }
            i2c_master_stop(cmd);
            ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);
            
            if (ret != ESP_OK) {
                return ret;
            }
        }
    }

    return ret;
}

/**
 * @brief Read random byte from EEPROM
 *
 * __________________________________________________________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | high_byte_addr + ack | low_byte_addr + ack | start | slave_addr + rd_bit + ack | read 1 byte + nack | stop |
 * |-------|---------------------------|----------------------|---------------------|-------|---------------------------|--------------------|------|
 *
 */
esp_err_t eeprom_read_byte(uint8_t deviceaddress, uint16_t eeaddress, uint8_t *byte) {
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress << 1) | EEPROM_WRITE_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, eeaddress << 8, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, eeaddress & 0xFF, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress << 1) | EEPROM_READ_ADDR, ACK_CHECK_EN);

    i2c_master_read_byte(cmd, byte, NACK_VAL);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

/**
 * @brief Read byte sequential from EEPROM
 *
 * ____________________________________________________________________________________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | high_byte_addr + ack | low_byte_addr + ack | start | slave_addr + rd_bit + ack | read byte_1 + ack | ... | read byte_x + nack | stop |
 * |-------|---------------------------|----------------------|---------------------|-------|---------------------------|-------------------|-----|--------------------|------|
 *
 */
esp_err_t eeprom_read(uint8_t deviceaddress, uint16_t eeaddress, uint8_t *data, size_t size) {
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress << 1) | EEPROM_WRITE_ADDR, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, eeaddress << 8, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, eeaddress & 0xFF, ACK_CHECK_EN);
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceaddress << 1) | EEPROM_READ_ADDR, ACK_CHECK_EN);

    if (size > 1) {
        i2c_master_read(cmd, data, size - 1, ACK_VAL);
    }

    i2c_master_read_byte(cmd, data + size - 1, NACK_VAL);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}
