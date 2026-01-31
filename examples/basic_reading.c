#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "as7265x.h"

// Define Pico 2W I2C pins
#define I2C_PORT i2c0
#define SDA_PIN  4
#define SCL_PIN  5

int main() {
    stdio_init_all();
    
    // Initialize I2C at 400kHz
    i2c_init(I2C_PORT, AS7265X_I2C_FREQ);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    as7265x_t sensor;
    
    printf("Initializing AS7265x Spectral Triad...\n");

    // Attempt to initialize the sensor chipset
    if (!as7265x_init(&sensor, I2C_PORT)) {
        printf("Failed to initialize sensor. Check wiring and Flash firmware!\n");
        while (1) tight_loop_contents();
    }

    printf("AS7265x Ready. Reading 18 channels...\n");

    float channels[18];
    const char* labels[18] = {
        "610nm(R)", "680nm(S)", "730nm(T)", "760nm(U)", "810nm(V)", "860nm(W)", // Master
        "560nm(G)", "585nm(H)", "645nm(I)", "705nm(J)", "900nm(K)", "940nm(L)", // Slave 1
        "410nm(A)", "435nm(B)", "460nm(C)", "485nm(D)", "510nm(E)", "535nm(F)"  // Slave 2
    };

    while (1) {

        while(!as7265x_is_data_ready(&sensor)) {
            sleep_ms(5);        
        }
        
        // Retrieve calibrated data (IEEE 754 floats)
        as7265x_get_all_calibrated(&sensor, channels);

        printf("\n--- Spectral Data ---\n");
        for (int i = 0; i < 18; i++) {
            printf("%s: %.4f  ", labels[i], channels[i]);
            if ((i + 1) % 3 == 0) printf("\n"); // Print 3 channels per line
        }

        // Delay based on integration time to avoid redundant polling
        // Default is 56ms, but we'll wait 1 second for readability
        sleep_ms(1000);
    }

    return 0;
}