#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 4
#define I2C_SCL 5

// LIS3DH Defines 
#define LIS3DH_ADDRESS 0x18 //I2C Address

#define CTRL_REG1 0x20 //Control Register 1 Address
#define CTRL_REG1_SETUP 0x97 //0x90 -- High Resolution Data Rate 1.344kHz + 0x07 -- Enable High Resolution mode and X Y Z axes

#define CTRL_REG4 0x23 //Control Register 4 Address
#define CTRL_REG4_SETUP 0x80 //0x80 -- Dont update registers until there is a reading

#define LIS3DH_X_AXIS_L 0x28
#define LIS3DH_Y_AXIS_L 0x2A
#define LIS3DH_Z_AXIS_L 0x2C

#define LIS3DH_X_AXIS_H 0x29
#define LIS3DH_Y_AXIS_H 0x2B
#define LIS3DH_Z_AXIS_H 0x2D

// LED Matrix MAX7219 Defines

void LIS3DH_Init() {

    uint8_t data[2];

    //Set data rate and enable axes
    data[0] = CTRL_REG1;
    data[1] = CTRL_REG1_SETUP;
    i2c_write_blocking(i2c_default, LIS3DH_ADDRESS, data, 2, false);

    data[0] = CTRL_REG4;
    data[1] = CTRL_REG4_SETUP;
    i2c_write_blocking(i2c_default, LIS3DH_ADDRESS, data, 2, false);

}

void LIS3DH_Get_Reading(uint8_t axis_register_L, uint8_t axis_register_H, float *calculated_acceleration) {

    uint8_t MSB;
    uint8_t LSB;
    uint16_t acceleration;

    i2c_write_blocking(i2c_default, LIS3DH_ADDRESS, &axis_register_L, 1, true);
    i2c_read_blocking(i2c_default, LIS3DH_ADDRESS, &LSB, 1, false);

    i2c_write_blocking(i2c_default, LIS3DH_ADDRESS, &axis_register_H, 1, true);
    i2c_read_blocking(i2c_default, LIS3DH_ADDRESS, &MSB, 1, false);

    acceleration = LSB | (MSB << 8);

    *calculated_acceleration = (float) ((int16_t) acceleration)  / 16000;

}

int main()
{
    stdio_init_all();

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(i2c_default, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

    printf("System Clock Frequency is %d Hz\n", clock_get_hz(clk_sys));
    printf("USB Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));
    // For more examples of clocks use see https://github.com/raspberrypi/pico-examples/tree/master/clocks

    float x_acceleration, y_acceleration, z_acceleration;

    LIS3DH_Init();

    //LIS3DH_Get_Reading(LIS3DH_X_AXIS_L, LIS3DH_X_AXIS_H, &x_acceleration);

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
        LIS3DH_Get_Reading(LIS3DH_X_AXIS_L, LIS3DH_X_AXIS_H, &x_acceleration);
        printf("%.3f\n", x_acceleration);
        LIS3DH_Get_Reading(LIS3DH_Y_AXIS_L, LIS3DH_Y_AXIS_H, &y_acceleration);
        printf("%.3f\n", y_acceleration);
        LIS3DH_Get_Reading(LIS3DH_Z_AXIS_L, LIS3DH_Z_AXIS_H, &z_acceleration);
        printf("%.3f\n", z_acceleration);
    }
}
