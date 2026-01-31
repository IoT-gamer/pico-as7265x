#ifndef AS7265X_H
#define AS7265X_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

/**
 * AS7265x I2C Configuration
 */
#define AS7265X_ADDR                0x49 // 7-bit Slave Address
#define AS7265X_I2C_FREQ            400000 // Fast Mode (400kHz)

/**
 * Physical I2C Registers (Hardware interface)
 */
#define AS7265X_PHYS_STATUS_REG     0x00 // Read-only: Status of I2C buffers
#define AS7265X_PHYS_WRITE_REG      0x01 // Write-only: Virtual Addr or Data
#define AS7265X_PHYS_READ_REG       0x02 // Read-only: Virtual Data response

/**
 * Status Register Bits
 */
#define AS7265X_STATUS_RX_VALID     0x01 // 1: Data available to be read
#define AS7265X_STATUS_TX_VALID     0x02 // 1: Write register occupied

/**
 * Virtual Register Map
 */
// Device Info & Config
#define AS7265X_HW_VERSION          0x00
#define AS7265X_FW_VERSION_H        0x02
#define AS7265X_FW_VERSION_L        0x03
#define AS7265X_CONFIG              0x04 // Sets Gain, Bank, Interrups
#define AS7265X_INT_TIME            0x05 // Integration time
#define AS7265X_TEMP                0x06 // Chip temperature
#define AS7265X_LED_CONFIG          0x07 // LED DRV and IND control

// Raw Data Registers (16-bit)
// Note: Actual channel (R-W, G-L, or A-F) depends on DEV_SEL
#define AS7265X_RAW_DATA_1_H        0x08 
#define AS7265X_RAW_DATA_1_L        0x09
#define AS7265X_RAW_DATA_2_H        0x0A
#define AS7265X_RAW_DATA_2_L        0x0B
#define AS7265X_RAW_DATA_3_H        0x0C
#define AS7265X_RAW_DATA_3_L        0x0D
#define AS7265X_RAW_DATA_4_H        0x0E
#define AS7265X_RAW_DATA_4_L        0x0F
#define AS7265X_RAW_DATA_5_H        0x10
#define AS7265X_RAW_DATA_5_L        0x11
#define AS7265X_RAW_DATA_6_H        0x12
#define AS7265X_RAW_DATA_6_L        0x13

// Calibrated Data Registers (32-bit IEEE 754 Floating Point)
#define AS7265X_CAL_DATA_1          0x14 // 4 bytes starting here
#define AS7265X_CAL_DATA_2          0x18
#define AS7265X_CAL_DATA_3          0x1C
#define AS7265X_CAL_DATA_4          0x20
#define AS7265X_CAL_DATA_5          0x24
#define AS7265X_CAL_DATA_6          0x28

// Device Selection 
#define AS7265X_DEV_SEL             0x4F
#define AS7265X_SEL_MASTER          0x00 // AS72651 (NIR)
#define AS7265X_SEL_SLAVE1          0x01 // AS72652 (VIS)
#define AS7265X_SEL_SLAVE2          0x02 // AS72653 (UV)

/**
 * Constants & Bitfield Values
 */
// Gain Settings
#define AS7265X_GAIN_1X             0x00
#define AS7265X_GAIN_3_7X           0x01
#define AS7265X_GAIN_16X            0x02
#define AS7265X_GAIN_64X            0x03

// Bank Settings
#define AS7265X_BANK_MODE_0         0x00 // 4 channels
#define AS7265X_BANK_MODE_1         0x01 // 4 channels
#define AS7265X_BANK_MODE_2         0x02 // All 6 channels (requires 2x integration time)

/**
 * LED Driver Current Settings (LED_DRV)
 */
#define AS7265X_DRV_CURR_12_5MA     0x00
#define AS7265X_DRV_CURR_25MA       0x01
#define AS7265X_DRV_CURR_50MA       0x02
#define AS7265X_DRV_CURR_100MA      0x03

/**
 * Indicator LED Current Settings (LED_IND)
 */
#define AS7265X_IND_CURR_1MA        0x00
#define AS7265X_IND_CURR_2MA        0x01
#define AS7265X_IND_CURR_4MA        0x02
#define AS7265X_IND_CURR_8MA        0x03


/**
 * API Handle
 */
typedef struct {
    i2c_inst_t *i2c;
    uint8_t address;
} as7265x_t;

/**
 * Initializes the AS7265x chipset.
 */
bool as7265x_init(as7265x_t *dev, i2c_inst_t *i2c_bus);

// Core Virtual Register Access

/**
 * Writes a byte to a virtual register.
 */
void as7265x_v_write(as7265x_t *dev, uint8_t virtual_reg, uint8_t value);

/**
 * Reads a byte from a virtual register.
 */
uint8_t as7265x_v_read(as7265x_t *dev, uint8_t virtual_reg);

// Data Acquisition

/**
 * Sets the gain of the sensor. 
 */
void as7265x_set_gain(as7265x_t *dev, uint8_t gain);

/**
 * Sets the integration time.
 * Value: 1-255. Integration time = <value> * 2.8ms
 */
void as7265x_set_integration_time(as7265x_t *dev, uint8_t time_value);

/**
 * Reads the temperature of a specific sensor in Celsius.
 * sensor_idx: AS7265X_SEL_MASTER, AS7265X_SEL_SLAVE1, or AS7265X_SEL_SLAVE2
 */
int8_t as7265x_get_temperature(as7265x_t *dev, uint8_t sensor_idx);

/**
 * Checks if data is ready to be read.
 * Returns true if the DATA_RDY bit is set.
 */
bool as7265x_is_data_ready(as7265x_t *dev);

/**
 * Retrieves all 18 raw channels (16-bit counts).
 * results array must be uint16_t[18]
 */
void as7265x_get_all_raw(as7265x_t *dev, uint16_t *results);

/**
 * Retrieves all 18 calibrated channels (IEEE 754 floats).
 * results array must be float[18]
 */
void as7265x_get_all_calibrated(as7265x_t *dev, float *results); // 18 channels

// LED Control

/**
 * Set current for the high-power Driver LED.
 */
void as7265x_set_led_drv_current(as7265x_t *dev, uint8_t sensor_idx, uint8_t current_level);

/**
 * Set current for the low-power Indicator LED.
 */
void as7265x_set_led_ind_current(as7265x_t *dev, uint8_t sensor_idx, uint8_t current_level);

/**
 * Enable or Disable the Driver LED.
 */
void as7265x_set_led_drv_enable(as7265x_t *dev, uint8_t sensor_idx, bool state);

/**
 * Enable or Disable the Indicator LED.
 */
void as7265x_set_led_ind_enable(as7265x_t *dev, uint8_t sensor_idx, bool state);

#endif