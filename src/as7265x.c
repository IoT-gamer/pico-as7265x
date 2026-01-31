#include "as7265x.h"
#include <stdio.h>
#include <string.h>

/**
 * Helper to read 4 bytes from a virtual address and convert to float (IEEE 754)
 * Multi-byte entities are Big Endian (MSB at lowest address).
 */
float as7265x_read_float(as7265x_t *dev, uint8_t base_virtual_reg) {
    uint32_t val = 0;
    for (int i = 0; i < 4; i++) {
        // Read bytes in order of ascending register addresses
        uint8_t byte = as7265x_v_read(dev, base_virtual_reg + i);
        val |= ((uint32_t)byte << (8 * (3 - i)));
    }
    
    float result;
    memcpy(&result, &val, 4);
    return result;
}

bool as7265x_is_data_ready(as7265x_t *dev) {
    uint32_t timeout = 500; // Cycles to poll 
    while (timeout--) {
        uint8_t config = as7265x_v_read(dev, AS7265X_CONFIG);
        if (config & 0x02) return true; // DATA_RDY bit is set 
        sleep_ms(10);
    }
    return false;
}

void as7265x_get_all_calibrated(as7265x_t *dev, float *results) {
    // Array of the three devices in the chipset
    uint8_t sensors[3] = {AS7265X_SEL_MASTER, AS7265X_SEL_SLAVE1, AS7265X_SEL_SLAVE2};
    
    // Ensure data is ready before starting capture
    if (!as7265x_is_data_ready(dev)) {
        printf("Warning: Data not ready, readings may be zero.\n");
    }

    for (int s = 0; s < 3; s++) {
        // Select the specific sensor
        as7265x_v_write(dev, AS7265X_DEV_SEL, sensors[s]);
        
        for (int ch = 0; ch < 6; ch++) {
            uint8_t reg = AS7265X_CAL_DATA_1 + (ch * 4);
            results[(s * 6) + ch] = as7265x_read_float(dev, reg);
        }
    }
}

void as7265x_set_gain(as7265x_t *dev, uint8_t gain) {
    if (gain > 0x03) gain = 0x03;
    uint8_t config = as7265x_v_read(dev, AS7265X_CONFIG);
    config &= ~(0x30); // Clear bits 5:4 
    config |= (gain << 4); // Set new gain bits 
    as7265x_v_write(dev, AS7265X_CONFIG, config);
}

void as7265x_set_integration_time(as7265x_t *dev, uint8_t time_value) {
    as7265x_v_write(dev, AS7265X_INT_TIME, time_value);
}

bool as7265x_init(as7265x_t *dev, i2c_inst_t *i2c_bus) {
    dev->i2c = i2c_bus;
    dev->address = AS7265X_ADDR; 

    // Verify Hardware Version
    uint8_t hw_version = as7265x_v_read(dev, AS7265X_HW_VERSION);
    if (hw_version != 0x40) {
        printf("Error: Incorrect HW Version 0x%02X\n", hw_version);
        return false;
    }

    // Software Reset 
    as7265x_v_write(dev, AS7265X_CONFIG, 0x80); 
    sleep_ms(1000); // 1s wait for Flash firmware reload

    // Verify Slaves
    uint8_t dev_sel_status = as7265x_v_read(dev, AS7265X_DEV_SEL);
    bool slave1_ready = (dev_sel_status & 0x10); // First slave bit
    bool slave2_ready = (dev_sel_status & 0x20); // Second slave bit
    
    if (!slave1_ready || !slave2_ready) {
        printf("Warning: Slaves missing. DEV_SEL: 0x%02X\n", dev_sel_status);
    }

    // // Default Configuration: Gain 16x (0x20) | Bank Mode 2 (0x08) = 0x28 
    // as7265x_v_write(dev, AS7265X_CONFIG, 0x28);
    // Gain 64x (bits 5:4 = 11) -> 0x30
    // Mode 2 (bits 3:2 = 10)   -> 0x08 (6-channel continuous)
    // Total = 0x38
    as7265x_v_write(dev, AS7265X_CONFIG, 0x38);
    // as7265x_set_integration_time(dev, 20); // ~56ms
    // Arduino uses 49 (50 * 2.8ms = 140ms)
    as7265x_set_integration_time(dev, 49);

    return true;
}

/**
 * Low-level physical status polling.
 */
static uint8_t as7265x_read_status(as7265x_t *dev) {
    uint8_t status;
    uint8_t reg = AS7265X_PHYS_STATUS_REG;
    i2c_write_blocking(dev->i2c, dev->address, &reg, 1, true);
    i2c_read_blocking(dev->i2c, dev->address, &status, 1, false);
    return status;
}

void as7265x_v_write(as7265x_t *dev, uint8_t virtual_reg, uint8_t value) {
    // Wait for Write flag to clear (allow write)
    while (as7265x_read_status(dev) & AS7265X_STATUS_TX_VALID) {
        sleep_ms(1); // Give sensor time to digest previous command
    }
    
    // Send Virtual Address (bit 7 set to 1 for write)
    uint8_t addr_buf[2] = { AS7265X_PHYS_WRITE_REG, (uint8_t)(virtual_reg | 0x80) };
    i2c_write_blocking(dev->i2c, dev->address, addr_buf, 2, false);

    // Wait for Write flag to clear
    while (as7265x_read_status(dev) & AS7265X_STATUS_TX_VALID) {
        sleep_ms(1);
    }
    
    // Send Data
    uint8_t data_buf[2] = { AS7265X_PHYS_WRITE_REG, value };
    i2c_write_blocking(dev->i2c, dev->address, data_buf, 2, false);
}

uint8_t as7265x_v_read(as7265x_t *dev, uint8_t virtual_reg) {
    // Wait for Write flag to clear
    while (as7265x_read_status(dev) & AS7265X_STATUS_TX_VALID) {
        sleep_ms(1);
    }

    // Send Virtual Address (bit 7 = 0 for read)
    uint8_t addr_buf[2] = { AS7265X_PHYS_WRITE_REG, virtual_reg };
    i2c_write_blocking(dev->i2c, dev->address, addr_buf, 2, false);

    // Wait for Read flag to be set (Data Ready)
    while (!(as7265x_read_status(dev) & AS7265X_STATUS_RX_VALID)) {
        sleep_ms(1); 
    }
    
    // 4. Read the data
    uint8_t read_cmd = AS7265X_PHYS_READ_REG;
    uint8_t data;
    i2c_write_blocking(dev->i2c, dev->address, &read_cmd, 1, true);
    i2c_read_blocking(dev->i2c, dev->address, &data, 1, false);
    return data;
}

int8_t as7265x_get_temperature(as7265x_t *dev, uint8_t sensor_idx) {
    // Select the target sensor (Master, Slave1, or Slave2)
    as7265x_v_write(dev, AS7265X_DEV_SEL, sensor_idx);
    
    // Read from the virtual temperature register
    uint8_t temp = as7265x_v_read(dev, AS7265X_TEMP);
    
    // Return -128 if there was a read error as per datasheet
    return (int8_t)temp; 
}

void as7265x_get_all_raw(as7265x_t *dev, uint16_t *results) {
    uint8_t sensors[3] = {AS7265X_SEL_MASTER, AS7265X_SEL_SLAVE1, AS7265X_SEL_SLAVE2};
    
    for (int s = 0; s < 3; s++) {
        as7265x_v_write(dev, AS7265X_DEV_SEL, sensors[s]);
        
        for (int ch = 0; ch < 6; ch++) {
            // Raw registers are consecutive 2-byte pairs: 0x08/0x09, 0x0A/0x0B, etc.
            uint8_t base_reg = AS7265X_RAW_DATA_1_H + (ch * 2);
            uint16_t high = as7265x_v_read(dev, base_reg);
            uint16_t low = as7265x_v_read(dev, base_reg + 1);
            
            // Reassemble 16-bit Big Endian value
            results[(s * 6) + ch] = (high << 8) | low;
        }
    }
}

void as7265x_set_led_drv_current(as7265x_t *dev, uint8_t sensor_idx, uint8_t current_level) {
    as7265x_v_write(dev, AS7265X_DEV_SEL, sensor_idx);
    uint8_t val = as7265x_v_read(dev, AS7265X_LED_CONFIG);
    val &= ~(0x30); // Clear bits 5:4
    val |= (current_level << 4); // Set current level
    as7265x_v_write(dev, AS7265X_LED_CONFIG, val);
}

void as7265x_set_led_drv_enable(as7265x_t *dev, uint8_t sensor_idx, bool state) {
    as7265x_v_write(dev, AS7265X_DEV_SEL, sensor_idx);
    uint8_t val = as7265x_v_read(dev, AS7265X_LED_CONFIG);
    if (state) val |= 0x08; // Set bit 3
    else val &= ~(0x08);
    as7265x_v_write(dev, AS7265X_LED_CONFIG, val);
}

void as7265x_set_led_ind_current(as7265x_t *dev, uint8_t sensor_idx, uint8_t current_level) {
    as7265x_v_write(dev, AS7265X_DEV_SEL, sensor_idx);
    uint8_t val = as7265x_v_read(dev, AS7265X_LED_CONFIG);
    val &= ~(0x06); // Clear bits 2:1
    val |= (current_level << 1); // Set current level
    as7265x_v_write(dev, AS7265X_LED_CONFIG, val);
}

void as7265x_set_led_ind_enable(as7265x_t *dev, uint8_t sensor_idx, bool state) {
    as7265x_v_write(dev, AS7265X_DEV_SEL, sensor_idx);
    uint8_t val = as7265x_v_read(dev, AS7265X_LED_CONFIG);
    if (state) val |= 0x01; // Set bit 0
    else val &= ~(0x01);
    as7265x_v_write(dev, AS7265X_LED_CONFIG, val);
}