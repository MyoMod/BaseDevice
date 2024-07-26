/**
 * @file comInterface.cpp
 * @author Leon Farchau (leon2225)
 * @brief 
 * @version 0.1
 * @date 26.07.2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

/************************************************************
 *  INCLUDES
 ************************************************************/
// c/c++ includes
#include <stdio.h>
#include "string.h"
#include <span>

// pico includes
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "pico/sync.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"

// myomod includes
#include "comInterface.h"
#include "i2c.h"
#include "commonRegisters.h"

// project includes
#include "specificRegisters.h"

/************************************************************
 *  DEFINES
 ************************************************************/
#define HOUT_BUFFER_SIZE MM_DEVICE_HOST_OUT_SIZE // size of the buffer in bytes for the H_OUT stream

#define HIN_BUFFER_SIZE MM_DEVICE_HOST_IN_SIZE // size of the buffer in bytes for the H_IN stream

#define CMD_UPDATE_CONFIG 0x01 // command to update the configuration
#define CMD_NEW_DATA 0x02      // command to process new HOut data
#define CMD_SYNC 0x03          // command that indicates a sync from host

#define CORE0_TO_1_ALARM 0
#define CORE1_TO_0_ALARM 1

enum class Core1To0Commands
{
    None,
    UpdateConfig,
    NewData,
    Sync
};


/************************************************************
 * Type definitions
 * **********************************************************/


/************************************************************
 * Variables
 * **********************************************************/
/**** Register interface ****/
StatusByte_t g_statusByte =
    {
    reserved2          : 0,
    realtime_warning   : 0,
    specific_warning   : 0,
    common_warning     : 0,
    reserved1          : 0,
    alignment_error    : 0,
    specific_error     : 0,
    common_error       : 0
};
CommonDeviceStatus_t g_commonDeviceStatus =
{
    reserved2              : 0,
    config_length_warning  : 0,
    reserved1              : 0,
    streamdirection_error  : 0,
    config_access_error    : 0,
    not_initialized_error  : 0
};
CommonDeviceInformation_t g_commonDeviceInfo =
{
    .protocol_version = {1, 0},
    .device_type = MM_DEVICE_TYPE,
    .identifier = MM_DEVICE_IDENTIFIER,
    .hostOut_size = MM_DEVICE_HOST_OUT_SIZE,
    .hostIn_size = MM_DEVICE_HOST_IN_SIZE,
    .device_version = MM_DEVICE_VERSION
};
CommonDeviceConfiguration_t g_commonDeviceConfiguration =
    {
    reserved    : 0,
    initialized : 0
};
DeviceSpecificStatus_t g_deviceSpecificStatus;
DeviceSpecificInfo_t g_deviceSpecificInfo;
DeviceSpecificConfiguration_t g_deviceSpecificConfiguration;

uint32_t g_regLength[] = {sizeof(g_statusByte), sizeof(g_commonDeviceStatus),
                          sizeof(g_commonDeviceInfo), sizeof(g_commonDeviceConfiguration),
                          sizeof(g_deviceSpecificStatus), sizeof(g_deviceSpecificInfo),
                          sizeof(g_deviceSpecificConfiguration)};
uint8_t *g_regPointers[] = {(uint8_t *)&g_statusByte, (uint8_t *)&g_commonDeviceStatus,
                            (uint8_t *)&g_commonDeviceInfo, (uint8_t *)&g_commonDeviceConfiguration,
                            (uint8_t *)&g_deviceSpecificStatus, (uint8_t *)&g_deviceSpecificInfo,
                            (uint8_t *)&g_deviceSpecificConfiguration};
AccessRights_t g_regAccessRights[] = {AccessRights_t::Read, AccessRights_t::Read,
                                      AccessRights_t::Read, AccessRights_t::ReadWrite,
                                      AccessRights_t::Read, AccessRights_t::Read,
                                      AccessRights_t::ReadWrite};

mutex_t g_regMutexes[NUM_REGISTERS];

static uint8_t g_HIn_Buffer[2][HIN_BUFFER_SIZE]; // Ping-Pong Buffer for the HIn data
static uint32_t g_HInBufferIndex = 0;             // index of the current HIn buffer being filled

static uint8_t g_HOut_Buffer[2][HOUT_BUFFER_SIZE]; // Ping-Pong Buffer for the HOut data

volatile static bool g_core0To1Index = false; // index of the current HIn buffer being transmitted from core0 to core1
volatile static bool g_core1To0Index = false; // index of the current HOut buffer being transmitted from core1 to core0

volatile static Core1To0Commands g_core1To0Command = Core1To0Commands::None; // command from core1 to core0

// true if the configuration register has been updated and the configuration needs to be updated
volatile static bool g_updateConfig = false;

// I2C Hardware interface
i2c_inst_t *g_i2c;
uint8_t g_i2cAddr;
uint32_t g_sdaPin;
uint32_t g_sclPin;

// Callbacks
void (*HOut_Callback)(const HostToDevice_t * const data);
void (*UpdateConfig_Callback)(DeviceSpecificConfiguration_t *config, DeviceSpecificConfiguration_t *oldConfig);
void (*Sync_Callback)(void);


/************************************************************
 * Function prototypes
 * **********************************************************/

void __isr multicoreFiFoIRQHandler(void);
void comInterfaceHandleConfigUpdate();

void core1_main(void);
int core1_init(void);
void core1_comInterfaceRun(void);
bool core1_WriteToRegister(void *buffer, uint32_t length, uint32_t registerName);
void core1_comInterfaceHandleHOutPDS(uint32_t bufferIndex);
bool core1_ReadFromRegister(void *buffer, uint32_t *length, uint32_t registerName);
bool __always_inline core1_ReadStatus(uint8_t *status);
void core1_comInterfaceHandleSync();
uint32_t core1_getRegLength(uint32_t registerIndex);

static void __isr core0_alarm_irq_callback();
static void __isr core1_alarm_irq_callback();
static inline void core1_force_alarmInterrupt();
static inline void core0_force_alarmInterrupt();


/************************************************************
 * Functions
 * **********************************************************/

/**
 * @brief Initializes the communication interface
 *
 * @param adc   pointer to the MAX11254 object
 * @return int32_t
 */
int32_t comInterfaceInit(cominterfaceConfiguration *config)
{
    // copy the configuration
    g_i2c = config->g_i2c;
    g_i2cAddr = config->g_i2cAddr;
    g_sdaPin = config->g_sdaPin;
    g_sclPin = config->g_sclPin;

    HOut_Callback = config->HOut_Callback;
    UpdateConfig_Callback = config->UpdateConfig_Callback;
    Sync_Callback = config->sync_callback;

    // initialize the mutexes
    for (uint32_t i = 0; i < NUM_REGISTERS; i++)
    {
        mutex_init(&g_regMutexes[i]);
    }

    // initialize the buffers
    if(HOUT_BUFFER_SIZE > 0)
    {
        memset(g_HOut_Buffer, 0, HOUT_BUFFER_SIZE * 2);
    }
    if(HIN_BUFFER_SIZE > 0)
    {
        memset(g_HIn_Buffer, 0, HIN_BUFFER_SIZE * 2);
    }

    // setup timers/alarms for cross core interrupt
    hardware_alarm_claim(CORE0_TO_1_ALARM);
    hardware_alarm_claim(CORE1_TO_0_ALARM);

    // register the alarm interrupt handler
    const uint irq_num = TIMER_IRQ_0 + CORE1_TO_0_ALARM;
    irq_set_exclusive_handler(irq_num, core0_alarm_irq_callback);
    irq_set_enabled(irq_num, true);
    // Enable interrupt in block and at processor
    hw_set_bits(&timer_hw->inte, 1u << CORE1_TO_0_ALARM);


    multicore_launch_core1(core1_main);


    return 0;
}

/**
 * @brief Sets the HIn buffer and handles the buffer management
 * 
 *
 * @param buffer        pointer to the buffer containing the data
 */
void comInterfaceSetHIn(DeviceToHost_t *buffer)
{  
    memcpy(&g_HIn_Buffer[g_HInBufferIndex], buffer, HIN_BUFFER_SIZE);
    g_core0To1Index = g_HInBufferIndex; 
    
    core0_force_alarmInterrupt();

    // check if there are new commands and issue them
    // now, so that there is no newstart of the buffers
    if (g_updateConfig)
    {
        comInterfaceHandleConfigUpdate();
        g_updateConfig = false;
    }

    g_HInBufferIndex = !g_HInBufferIndex; // switch ping-pong buffer
}

/**
 * @brief Returns the device specific status
 *
 * @param status
 */
void comInterfaceGetStatus(DeviceSpecificStatus_t *status)
{
    mutex_enter_blocking(&g_regMutexes[(uint)DeviceRegisterType::DeviceSpecificStatus]);
    memcpy(status, &g_deviceSpecificStatus, sizeof(DeviceSpecificStatus_t));
    mutex_exit(&g_regMutexes[(uint)DeviceRegisterType::DeviceSpecificStatus]);
}

/**
 * @brief Sets the device specific status
 *
 * @param status
 */
void comInterfaceSetStatus(DeviceSpecificStatus_t *status, bool generateWarning, bool generateError)
{
    mutex_enter_blocking(&g_regMutexes[(uint)DeviceRegisterType::DeviceSpecificStatus]);
    memcpy(&g_deviceSpecificStatus, status, sizeof(DeviceSpecificStatus_t));
    mutex_exit(&g_regMutexes[(uint)DeviceRegisterType::DeviceSpecificStatus]);


    mutex_enter_blocking(&g_regMutexes[(uint)DeviceRegisterType::Status]);
    g_statusByte.specific_warning = generateWarning;
    g_statusByte.specific_error = generateError;
    mutex_exit(&g_regMutexes[(uint)DeviceRegisterType::Status]);
    
}

void __isr core0_alarm_irq_callback()
{
    // Clear the timer IRQ
    timer_hw->intr = 1u << CORE1_TO_0_ALARM;
    // Clear any forced IRQ
    hw_clear_bits(&timer_hw->intf, 1u << CORE1_TO_0_ALARM);

    
    // Handle the HOut buffer
    switch (g_core1To0Command)
    {
    case Core1To0Commands::UpdateConfig:
        g_updateConfig = true;
        break;
    case Core1To0Commands::NewData:
        if (HOut_Callback != NULL)
        {
            HOut_Callback(reinterpret_cast<HostToDevice_t *>(&g_HOut_Buffer[g_core1To0Index]));
        }
        break;
    case Core1To0Commands::Sync:
        if (Sync_Callback != NULL)
        {
            Sync_Callback();
        }
        break;
    default:
        __breakpoint();
        break;
    }
}

/**
 * @brief Handles the configuration update
 *
 */
void comInterfaceHandleConfigUpdate()
{
    static DeviceSpecificConfiguration_t oldConfig;
    static DeviceSpecificConfiguration_t currentConfig;

    // Read the new configuration from the register
    mutex_enter_blocking(&g_regMutexes[(uint)DeviceRegisterType::DeviceSpecificConfiguration]);
    memcpy(&currentConfig, &g_deviceSpecificConfiguration, sizeof(DeviceSpecificConfiguration_t));
    mutex_exit(&g_regMutexes[(uint)DeviceRegisterType::DeviceSpecificConfiguration]);

    // call the callback if it is set
    if (UpdateConfig_Callback != NULL)
    {
        UpdateConfig_Callback(&currentConfig, &oldConfig);
    }

    // copy the new configuration to the old configuration so that the callback
    //  can compare the two in the next call
    memcpy(&oldConfig, &currentConfig, sizeof(DeviceSpecificConfiguration_t));
}

/**
 * @brief Forces an interrupt on core1
 * 
 */
static inline void core0_force_alarmInterrupt()
{
    hw_set_bits(&timer_hw->intf, 1u << CORE0_TO_1_ALARM);
}

/******** CORE 1 ********************************/
void core1_main(void)
{
    core1_init();

    while (1)
    {
        core1_comInterfaceRun();
    }
}

int core1_init(void)
{
    // init i2c
    uint32_t longestRegisterLength = 0;
    for (uint32_t i = 0; i < NUM_REGISTERS; i++)
    {
        if (g_regLength[i] > longestRegisterLength)
        {
            longestRegisterLength = g_regLength[i];
        }
    }

    i2cInitConfiguration_t i2cConfig;
    i2cConfig.i2c = g_i2c;
    i2cConfig.i2cAddr = g_i2cAddr;
    i2cConfig.sdaPin = g_sdaPin;
    i2cConfig.sclPin = g_sclPin;

    i2cConfig.longestRegisterLength = longestRegisterLength;
    i2cConfig.HIn_pdsLength = HIN_BUFFER_SIZE;
    i2cConfig.HOut_pdsBuffer = g_HOut_Buffer;
    i2cConfig.HOut_pdsLength = HOUT_BUFFER_SIZE;

    i2cConfig.H_Out_NotifyPdsBufferFull = core1_comInterfaceHandleHOutPDS;
    i2cConfig.H_In_GetRegisterCallback = core1_ReadFromRegister;
    i2cConfig.H_In_GetStatusCallback = core1_ReadStatus;
    i2cConfig.H_Out_RegisterCallback = core1_WriteToRegister;
    i2cConfig.getRegisterLength = core1_getRegLength;

    i2cConfig.sync_callback = core1_comInterfaceHandleSync;
    I2C_Init(&i2cConfig);

    // setup timers/alarms for cross core interrupt    
    const uint irq_num = TIMER_IRQ_0 + CORE0_TO_1_ALARM;
    irq_set_exclusive_handler(irq_num, core1_alarm_irq_callback);
    irq_set_enabled(irq_num, true);
    // Enable interrupt in block and at processor
    hw_set_bits(&timer_hw->inte, 1u << CORE0_TO_1_ALARM);

    return 0;
}

void core1_comInterfaceRun(void)
{
    // check if there is a HIn buffer to send
    if (multicore_fifo_rvalid())
    {
        tight_loop_contents();
    }
}

static void __isr core1_alarm_irq_callback()
{
    // Clear the timer IRQ
    timer_hw->intr = 1u << CORE0_TO_1_ALARM;
    // Clear any forced IRQ
    hw_clear_bits(&timer_hw->intf, 1u << CORE0_TO_1_ALARM);

    
    // Handle the HOut buffer
    uint32_t length = HIN_BUFFER_SIZE; // length in bytes
    I2C_send_H_In_PDSData(reinterpret_cast<const uint8_t*> (g_HIn_Buffer[g_core0To1Index]), length);
}

/**
 * @brief Writes data to a register.
 *  This function writes the provided data to the specified register. It performs various checks to ensure the validity of the operation.
 *  If the length of the data, the register name, and the access rights are all valid, the data is copied to the register.
 *  If the register name is equal to or greater than REG_DeviceSpecificStatus, it informs core0 that the configuration has changed.
 *  If any of the checks fail, the error bit in the status byte is set and the error state is updated.
 *
 * @param buffer Pointer to the data buffer.
 * @param length Length of the data buffer.
 * @param registerName The name/index of the register to write to.
 * @return void
 */
bool core1_WriteToRegister(void *buffer, uint32_t length, uint32_t registerName)
{
    // check if the length is valid
    bool valid = true;
    valid &= (length == g_regLength[registerName]);
    valid &= (registerName < NUM_REGISTERS);
    valid &= (g_regAccessRights[registerName] == AccessRights_t::Write) ||
             (g_regAccessRights[registerName] == AccessRights_t::ReadWrite);

    if (valid)
    {
        // copy the data to the register
        mutex_enter_blocking(&g_regMutexes[registerName]);
        memcpy(g_regPointers[registerName], buffer, length);
        mutex_exit(&g_regMutexes[registerName]);

        // inform core0 that the configuration has changed
        if (registerName = (uint)DeviceRegisterType::DeviceSpecificConfiguration)
        {
            g_core1To0Command = Core1To0Commands::UpdateConfig;
            core1_force_alarmInterrupt();
        }
    }
    else
    {
        // set the error bit in the status byte
        mutex_enter_blocking(&g_regMutexes[(uint)DeviceRegisterType::CommonDeviceStatus]);
        g_commonDeviceStatus.config_access_error = 1;
        mutex_exit(&g_regMutexes[(uint)DeviceRegisterType::CommonDeviceStatus]);
        mutex_enter_blocking(&g_regMutexes[(uint)DeviceRegisterType::Status]);
        g_statusByte.common_error = 1;
        mutex_exit(&g_regMutexes[(uint)DeviceRegisterType::Status]);
    }
    return valid;
}

/**
 * @brief This function should be called from i2c when new data is
 *          available for the HOut stream.
 *
 * @param data      pointer to the data buffer
 * @param length    length of the data buffer
 */
void core1_comInterfaceHandleHOutPDS(uint32_t bufferIndex)
{
    // command core1 to send the buffer
    g_core1To0Index = bufferIndex;
    g_core1To0Command = Core1To0Commands::NewData;
    core1_force_alarmInterrupt();
}

/**
 * @brief This function should be called from i2c when a sync is received.
 *
 */
void core1_comInterfaceHandleSync()
{
    g_core1To0Command = Core1To0Commands::Sync;
    core1_force_alarmInterrupt();
}

/**
 * @brief Returns the length of a register.
 *
 * @param registerIndex The index of the register.
 * @return uint32_t The length of the register.
 */
uint32_t core1_getRegLength(uint32_t registerIndex)
{
    assert(registerIndex < NUM_REGISTERS);
    return g_regLength[registerIndex];
}

/**
 * @brief Reads data from a register.
 *
 * This function reads data from a specified register and copies it into the provided buffer.
 * It performs various checks to ensure the validity of the operation, such as checking the length,
 * the register name, and the access rights. If the operation is valid, the data is copied from the register
 * into the buffer. If the operation is invalid, error bits in the status byte are set accordingly.
 * The caller needs to read the register using mutexes to ensure that the data is not changed while it is being read.
 *
 * @param buffer The content of the register is copied into this buffer.
 * @param length Pointer to the length of the data to be read, if 0 the length will be written into the pointer.
 * @param registerName The name of the register to read from.
 * @return true if the operation is successful, false otherwise.
 */
bool core1_ReadFromRegister(void *buffer, uint32_t *length, uint32_t registerName)
{
    // check if the length is valid
    bool valid = true;
    if (*length == 0)
    {
        *length = g_regLength[registerName];
    }
    else
    {
        valid &= (*length == g_regLength[registerName]);
    }
    valid &= (registerName < NUM_REGISTERS);
    valid &= (g_regAccessRights[registerName] == AccessRights_t::Read) ||
             (g_regAccessRights[registerName] == AccessRights_t::ReadWrite);

    if (valid)
    {
        // copy the data from the register
        memcpy(buffer, g_regPointers[registerName], *length);
    }
    else
    {
        // set the error bit in the status byte
        mutex_enter_blocking(&g_regMutexes[(uint)DeviceRegisterType::CommonDeviceStatus]);
        g_commonDeviceStatus.config_access_error = 1;
        mutex_exit(&g_regMutexes[(uint)DeviceRegisterType::CommonDeviceStatus]);
        mutex_enter_blocking(&g_regMutexes[(uint)DeviceRegisterType::Status]);
        g_statusByte.common_error = 1;
        mutex_exit(&g_regMutexes[(uint)DeviceRegisterType::Status]);
    }
    return valid;
}

/**
 * @brief Reads the status byte from the register and checks the access rights.
 *
 *
 * @param status Pointer to the variable where the status byte will be stored.
 * @return true if the access rights are valid and the status byte is successfully read,
 *         false otherwise.
 */
bool __always_inline core1_ReadStatus(uint8_t *status)
{
    // copy the data from the register
    *status = *(uint8_t *)g_regPointers[(uint)DeviceRegisterType::Status];

    return true;
}

/**
 * @brief Forces an interrupt on core0
 *
 */
static inline void core1_force_alarmInterrupt()
{
    hw_set_bits(&timer_hw->intf, 1u << CORE1_TO_0_ALARM);
}
