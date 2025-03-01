#pragma once
/**
 * @file comInterface.h
 * @author Leon Farchau (leon2225)
 * @brief 
 * @version 0.1
 * @date 03.01.2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

/************************************************************************************
 * INCLUDES
 ************************************************************************************/
#include <stdint.h>
#include "hardware/i2c.h"

#include "specificRegisters.h"

/************************************************************************************
 * DEFINES
 ************************************************************************************/

/************************************************************************************
 * Data types
 * *********************************************************************************/
struct cominterfaceConfiguration
{
    // I2C Hardware interface
    i2c_inst_t* g_i2c;
    uint8_t g_i2cAddr;
    uint32_t g_sdaPin;
    uint32_t g_sclPin;

    // Callbacks
    void (*HOut_Callback)(const HostToDevice_t* const data);
    void (*UpdateConfig_Callback)(DeviceSpecificConfiguration_t* config, DeviceSpecificConfiguration_t* oldConfig);
    void (*sync_callback)(void);
};


/************************************************************************************
 * Functions
 * *********************************************************************************/

int32_t comInterfaceInit(cominterfaceConfiguration* config);
void comInterfaceSetHIn(DeviceToHost_t *data);
void comInterfaceGetStatus(DeviceSpecificStatus_t *status);
void comInterfaceSetStatus(DeviceSpecificStatus_t *status, bool generateWarning, bool generateError);
