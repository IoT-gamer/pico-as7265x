# AS7265x Spectral Triad Library for Raspberry Pi Pico

A lightweight, C-based library for the **[ams AS7265x](https://cdn.sparkfun.com/assets/c/2/9/0/a/AS7265x_Datasheet.pdf)** (18-channel spectral sensor) designed specifically for the **Raspberry Pi Pico / Pico 2W** SDK. This library handles the complex virtual register protocol, multi-sensor calibration, and chipset initialization required for the [**SparkFun Spectral Triad board**](https://github.com/sparkfun/Qwiic_Spectral_Sensor_AS7265x).

## Features
* **18-Channel Support:** Access all channels from 410nm to 940nm.

* **Calibrated Output:** Returns 32-bit floating-point values (counts/$\mu W/cm^{2}$) using the IEEE 754 standard.

* **Raw Data Access:** 16-bit raw ADC counts for high-speed polling.+1Diagnostics: On-chip temperature sensing for all three sensors.

* **Configurable:** Easy adjustment of Gain and Integration Time.

* **FetchContent Ready:** Built to be included as a CMake dependency in other Pico projects.

## Wiring (Pico 2W)
| AS7265x Pin | Pico Pin (Example) | GPIO | Function            |
| :---        | :---           | :---     | :---                 |
| 3.3V        | 3V3 (Pin 36)   |    -     | Power (2.7V to 3.6V) |
| GND         | GND (Pin 38)   |    -     | Ground               |
| SDA         | GP4 (Pin 6)    |  GPIO 4  | I2C0 Data            |
| SCL         | GP5 (Pin 7)    |  GPIO 5  | I2C0 Clock           |

## Installation via CMake
```cmake
include(FetchContent)

FetchContent_Declare(
  as7265x_pico
  GIT_REPOSITORY https://github.com/IoT-gamer/pico-as7265x.git
  GIT_TAG        main # or specify a release tag
)
FetchContent_MakeAvailable(as7265x_pico)

# Link to your executable
target_link_libraries(your_project_name PRIVATE as7265x_pico)
```

## Quick Start Example
```c
#include "as7265x.h"

int main() {
    stdio_init_all();
    i2c_init(i2c0, 400000); // 400kHz Fast Mode
    
    as7265x_t sensor;
    if (as7265x_init(&sensor, i2c0)) {
        float channels[18];
        while (1) {
            while(!as7265x_is_data_ready(&sensor)) {
                sleep_ms(5);        
            }
            as7265x_get_all_calibrated(&sensor, channels);
            printf("Visible (610nm): %.2f\n", channels[0]);
            sleep_ms(1000);
        }
    }
}
```

## Technical Notes

* **Virtual Registers:** This library implements the ams "Smart Interface," which uses a physical $I^{2}C$ status register ($0x00$) to proxy commands to virtual registers.

* **Integration Time:** In 18-channel mode (Bank Mode 2), the sensor requires **two** integration cycles to capture all data.

* **Firmware:** This library assumes the AS72651 has valid firmware loaded in its companion serial flash.

## License
This library is released under the MIT License. See `LICENSE` for details.

## Acknowledgements & References
* [AS7265x Datasheet](https://cdn.sparkfun.com/assets/c/2/9/0/a/AS7265x_Datasheet.pdf) by ams OSRAM
* [SparkFun Qwiic AS7265x Arduino Library](https://github.com/sparkfun/SparkFun_AS7265x_Arduino_Library) by SparkFun Electronics