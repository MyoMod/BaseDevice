
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

#include "specificRegisters.h"
#include "comInterface.h"

/************************************************************
 *  DEFINES
 ************************************************************/
#define I2C_UNIT i2c1
#define I2C_ADDR 0x18
#define SDA_PIN 2
#define SCL_PIN 3

/************************************************************
 * Type definitions
 * **********************************************************/


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
void dataCallback(const HostToDevice_t * const data);
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

    return 0;
}

void setup()
{
    stdio_init_all();

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

        // Read data in a async way
        static DeviceToHost_t data;
        data[0] = 0x01;
        comInterfaceSetHIn(&data);
    }
}

void dataCallback(const HostToDevice_t * const data)
{
    // handle new data
    for (size_t bar = 0; bar < data->size(); bar++)
    {
        printf("Bar %d: %d\n", bar, data->at(bar));
    }
}

void configCallback(DeviceSpecificConfiguration_t* config, DeviceSpecificConfiguration_t* oldConfig)
{
    g_config = config;
}