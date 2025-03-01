#pragma once

/**
 * @brief 
 * 
 */

/************************************************************************************
 * INCLUDES
 ************************************************************************************/
#include <stdint.h>
#include "charArray.h"

/************************************************************************************
 * DEFINES
 ************************************************************************************/
#define MM_DEVICE_VERSION       {0, 1}
#define MM_DEVICE_IDENTIFIER    idArr("BaseDev  1")
#define MM_DEVICE_TYPE          idArr("BaseDevice")
#define MM_DEVICE_HOST_OUT_SIZE 1
#define MM_DEVICE_HOST_IN_SIZE  1


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
    // Example:
    uint8_t DisplayDetected : 8 = 1;
};

/**
 * @brief Device specific information
 * 
 */
struct __attribute__((packed)) DeviceSpecificInfo_t
{
    // Example:
    uint8_t DeviceSpecificInfo[10] = "BarDispl";
};

/**
 * @brief Device specific configuration
 * 
 */
struct DeviceSpecificConfiguration_t
{
    // Example:
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