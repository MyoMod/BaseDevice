#pragma once

/**
 * @brief 
 * 
 */

/************************************************************************************
 * INCLUDES
 ************************************************************************************/
#include <stdint.h>

/************************************************************************************
 * DEFINES
 ************************************************************************************/

/************************************************************************************
 * PROTOTYPES
 * *********************************************************************************/

/************************************************************************************
 * DATA TYPES
 * *********************************************************************************/

/**
 * @brief Device specific status
 * 
 */
struct __attribute__((packed)) DeviceSpecificStatus_t
{
    uint8_t DisplayDetected : 8 = 1;
};

/**
 * @brief Device specific information
 * 
 */
struct __attribute__((packed)) DeviceSpecificInfo_t
{
    uint8_t DeviceSpecificInfo[10] = "BarDispl";
};

/**
 * @brief Device specific configuration
 * 
 */
struct DeviceSpecificConfiguration_t
{
    uint32_t BarColors[7] =
    {
        0x00ff00,
        0xffff00,
        0xff0000,
        0xff00ff,
        0x0000ff,
        0x00ffff,
        0x000000
    };
};