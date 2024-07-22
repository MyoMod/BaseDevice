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

#include <array>
// Define types for Host -> Device (HOut)
typedef std::array<uint8_t, 7> HostToDevice_t;
// Define types for Device -> Host (HIn)
typedef std::array<uint8_t, 4> DeviceToHost_t;

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