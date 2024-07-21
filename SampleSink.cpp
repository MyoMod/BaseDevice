
/**
 * @file SampleSink.cpp
 * @author Leon Farchau (leon2225)
 * @brief 
 * @version 0.1
 * @date 03.01.2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

/************************************************************
 *  INCLUDES
 ************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "hardware/gpio.h"
#include "pico_unicorn.hpp"
#include "display.hpp"
#include "debug_pins.h"

#include "specificRegisters.h"
#include "comInterface.h"

/************************************************************
 *  DEFINES
 ************************************************************/

const uint32_t BUTTON_A = pimoroni::PicoUnicorn::A;
const uint32_t BUTTON_B = pimoroni::PicoUnicorn::B;
const uint32_t BUTTON_X = pimoroni::PicoUnicorn::X;
const uint32_t BUTTON_Y = pimoroni::PicoUnicorn::Y;

const uint32_t BUTTON_PINS[] = {
    BUTTON_A,
    BUTTON_B,
    BUTTON_X,
    BUTTON_Y};

/************************************************************
 * Type definitions
 * **********************************************************/
#define I2C_UNIT i2c1
#define I2C_ADDR 0x18
#define SDA_PIN 2
#define SCL_PIN 3


/************************************************************
 * Variables
 * **********************************************************/
DeviceSpecificConfiguration_t* g_config;
volatile bool g_sync = false;

/************************************************************
 * Function prototypes
 * **********************************************************/
void setup();
void asyncLoop();
void dataCallback(void* data, uint32_t length);
void syncCallback();
void configCallback(DeviceSpecificConfiguration_t* config, DeviceSpecificConfiguration_t* oldConfig);

/************************************************************
 * Functions
 * **********************************************************/

int main()
{
    setup();

    while (true)
    {
        asyncLoop();
    }

    printf("done\n");

    return 0;
}

void setup()
{
    stdio_init_all();

    gpio_init(DEBUG_PIN1);
    gpio_init(DEBUG_PIN2);
    gpio_init(DEBUG_PIN3);
    gpio_set_dir(DEBUG_PIN1, GPIO_OUT);
    gpio_set_dir(DEBUG_PIN2, GPIO_OUT);
    gpio_set_dir(DEBUG_PIN3, GPIO_OUT);
    gpio_put(DEBUG_PIN1, 1);
    gpio_put(DEBUG_PIN2, 1);
    gpio_put(DEBUG_PIN3, 1);

    for (size_t i = 0; i < 4; i++)
    {
        gpio_init(BUTTON_PINS[i]);
        gpio_set_dir(BUTTON_PINS[i], GPIO_IN);
    }

    display_init(0.7f);

    cominterfaceConfiguration config;
    config.g_i2c = I2C_UNIT;
    config.g_i2cAddr = I2C_ADDR;
    config.g_sdaPin = SDA_PIN;
    config.g_sclPin = SCL_PIN;
    config.HOut_Callback = dataCallback;
    config.UpdateConfig_Callback = configCallback;
    config.sync_callback = syncCallback;

    comInterfaceInit(&config);
}

void syncCallback()
{
    g_sync = true;
}

void asyncLoop()
{    
    if (g_sync)
    {
        g_sync = false;
        gpio_put(DEBUG_PIN1, 1);
        uint8_t data;
        for (size_t i = 0; i < 4; i++)
        {
            data= !gpio_get(BUTTON_PINS[i]);
            comInterfaceAddSample(&data, i);
        }
        
        gpio_put(DEBUG_PIN1, 0);
    }
}

void dataCallback(void* data, uint32_t length)
{
    uint8_t *values = (uint8_t*) data;
    uint32_t barNum = MIN(length, 7);

    for (size_t bar = 0; bar < barNum; bar++)
    {
        display_drawBar(bar, values[bar]);
    }
}

void configCallback(DeviceSpecificConfiguration_t* config, DeviceSpecificConfiguration_t* oldConfig)
{
    g_config = config;
}