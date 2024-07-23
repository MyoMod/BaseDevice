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
enum class DeviceRegisterType
{
	Status = 0,
	CommonDeviceStatus,
	CommonDeviceInformation,
	CommonDeviceConfiguration,
	DeviceSpecificStatus,
	DeviceSpecificInformation,
	DeviceSpecificConfiguration,
    n
};
#define NUM_REGISTERS ((int)DeviceRegisterType::n)

/**
 * @brief Control byte for the communication interface
 * 
 */
struct __attribute__((packed)) ControlByte_t
{
    uint32_t PDS_nRegister : 1;
    uint32_t HostIn_nHostOut : 1;
    
    uint32_t ADDR : 6; // used only for register access, otherwise reserved
};

/**
 * @brief Status byte for the communication interface
 * 
 */
struct __attribute__((packed)) StatusByte_t
{
    bool reserved2          : 1;
    bool realtime_warning   : 1;
    bool specific_warning   : 1;
    bool common_warning     : 1;
    bool reserved1          : 1;
    bool alignment_error    : 1;
    bool specific_error     : 1;
    bool common_error       : 1;
};

/**
 * @brief Common Device Status
 * 
 */
struct __attribute__((packed)) CommonDeviceStatus_t
{
    bool reserved2              : 3;
    bool config_length_warning  : 1;
    bool reserved1              : 1;
    bool streamdirection_error  : 1;
    bool config_access_error    : 1;
    bool not_initialized_error  : 1;
};

/**
 * @brief Common Device Information
 * 
 */
struct __attribute__((packed)) CommonDeviceInformation_t
{
    uint8_t             protocol_version[2]; // Byte 0+1
    std::array<char,10> device_type;
    std::array<char,10> identifier;
    uint16_t            hostOut_size;
    uint16_t            hostIn_size;
    uint8_t             device_version[2];
};


/**
 * @brief Common Device Configuration
 * 
 */
struct __attribute__((packed)) CommonDeviceConfiguration_t
{
    uint8_t reserved : 7;
    uint8_t initialized : 1;
};

/**
 * @brief Accessrights for the registers
 * 
 */
enum class AccessRights_t
{
    NoAccess = 0,
    Read = 1,
    Write = 2,
    ReadWrite = 3
};