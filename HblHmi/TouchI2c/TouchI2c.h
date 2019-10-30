/*
 * TouchI2c.h
 *
 *  Created on: Nov 29, 2016
 *      Author: bhimahv
 */
//***************************************************************************************
#ifndef SOURCE_APPSPECIFIC_DRIVER_TOUCHI2C_TOUCHI2C_H_
    #define SOURCE_APPSPECIFIC_DRIVER_TOUCHI2C_TOUCHI2C_H_

#include "SystemConfig.h"

#if(TOUCH_I2C_PROTOCOL == ENABLED)
#include "TouchI2c_prm.h"

#ifndef GESE_SETTING_TOUCH_PARAMETERS
        #define GESE_SETTING_TOUCH_PARAMETERS  DISABLED
#endif //This macro closing for GESE_SETTING_TOUCH_PARAMETERS
#if (GESE_SETTING_TOUCH_PARAMETERS == ENABLED)
//! define number of keys in the system allocate max as per GESE Tool
#define TOUCHI2C_TOTAL_NUMBER_OF_KEYS           40
//!---------------------------------------------------------------------------------------------------------------------
//! define number of touchi2 cypress controllers. as per GESE Tool
#define TOUCHI2C_TOTAL_NUMBER_OF_SLAVE_DEVICES             2

#endif
#define TOUCHI2C_SENSORS_SELF_TYPE                  0
#define TOUCHI2C_SENSORS_MUTUAL_TYPE                1

#define TOUCHI2C_MAJOR_VERSION       03
#define TOUCHI2C_VALIDATION_VERSION  00
#define TOUCHI2C_TEST_VERSION        00


typedef enum
{
    TOUCHI2C_STATE_INIT,
    TOUCHI2C_STATE_READ_WRITE,
    TOUCHI2C_STATE_REQUEST_READ,
    TOUCHI2C_STATE_WAIT_WRITE_COMPLETION,
    TOUCHI2C_KEY_STATE_ACTUAL_WRITE,
    TOUCHI2C_STATE_REQUEST_DATA
}TOUCHI2C_FM_STATE_TYPE;

typedef enum
{
	TOUCHI2C_READ_COMMAND,
	TOUCHI2C_WRITE_COMMAND
}TOUCHI2C_STATE_RW_CONTROL_TYPE;

typedef enum
{
    TOUCHI2C_IDLE,
    TOUCHI2C_ERROR,
    TOUCHI2C_BUSY,
    TOUCHI2C_IN_PROGRESS,
    TOUCHI2C_SUCCESS,
    TOUCHI2C_TIMEOUT,
    TOUCHI2C_RETRY
}TOUCHI2C_HW_STATE;

typedef enum
{
	TOUCHI2C_CMD_IDLE,
	TOUCHI2C_CMD_STORE_DEV_CONFIG,
	TOUCHI2C_CMD_STORE_CAPSENSE_CONFIG,
	TOUCHI2C_CMD_LOAD_DEVICE_CONFIG=5,
	TOUCHI2C_CMD_LOAD_CAPSENSE_CONFIG,
	TOUCHI2C_CMD_CLEAR_HOST_INT,
	TOUCHI2C_CMD_CLEAR_LATCH_STS=9,
	TOUCHI2C_CMD_ALP_FILTER_RESET,
	TOUCHI2C_CMD_SOFTWARE_RESET,
	TOUCHI2C_CMD_CLEAR_ERR_STATUS,
	TOUCHI2C_CMD_CLEAR_TOUCH_DELTA_MAX,
	TOUCHI2C_CMD_SUSPEND_CAPSENSE_SCANNING,
	TOUCHI2C_CMD_RESUME_CAPSENSE_SCANNING,
	TOUCHI2C_CMD_LOW_POWER_MODE,
	TOUCHI2C_CMD_ACTIVE_MODE,
	TOUCHI2C_CMD_SEND_TX8_DATA,
	TOUCHI2C_CMD_STOP_TX8_DATA,
	TOUCHI2C_CMD_RESET_SENSOR_BSLN,
	TOUCHI2C_CMD_WDT_RESET_CHECK,
	TOUCHI2C_CMD_UPDATE_CUR_DELTA,
	TOUCHI2C_CMD_UPDATE_MAX_DELTA,
	TOUCHI2C_CMD_LOAD_DEVICE_INFO,
	TOUCHI2C_CMD_UPDATE_AVG_DATA,
	TOUCHI2C_CMD_UPDATE_ALP_DATA
}TOUCHI2C_PROTOCOL_COMMANDS;

typedef PACKED struct
{
    //! Register content TouchI2C Slave device
    unsigned char TouchI2c_Slave_Device;

    //!Slave address of TouchI2C Devices
    unsigned char slaveAddress;

    //! MSB Slave offset address
    unsigned char slaveOffsetMSB;

    //! LSB Slave offset address
    unsigned char slaveOffsetLSB;

    //!Read or write I2c Operation
    TOUCHI2C_STATE_RW_CONTROL_TYPE i2c_read_write;

    //! number of Read or write bytes
    unsigned char number_Of_read_write_bytes;

    //! pointer to read or write memory location
    unsigned char* src_dst_buffer_ptr;

    //! TouchI2C State define
    TOUCHI2C_HW_STATE touchI2C_Hw_State;

    //!requesting send command continuously

    unsigned char Send_Command_Continously;

    //!Command request
    unsigned char IscmdRequest;

    //!If device is not responding or Busy state  wait until  this delay before initializing   I2C Bus
    unsigned short int TouchI2c_Hangup_Delay_Timeout;

    //! Touchi2C  Finite states definition
     TOUCHI2C_FM_STATE_TYPE  TouchI2c_FM_State;

}TOUCHI2C_COMMAND_STRUCT;

typedef PACKED struct
{
   uint8 ctrlCommand;

   /* This register is used to pass the operand of the command or the
   CRC8 byte of commands like soft reset
   */
   uint8 ctrlCommandOperand;

   /* Indicates the command number. Useful in case if same command has to be sent multiple times */
   uint8 ctrlCommandCntr;

   /* This register indicates status of the last command executed */
   uint8 ctrlCommandEcho;

   /* Indicates the counter value received in the last command */
   uint8 ctrlCommandEchoCntr;

   /* All the below registers are Read-Only parameters for the Master */

   /* This register indicates the error caused by the command execution */
   uint8 ctrlCmdError;

   /* This register indicates the last error encountered */
   uint8 errorStatus;

   /* This register shows the number of errors encountered */
   uint8 errorCounter;

   /* error code of class-b firmware */
   uint16 classBErrorStatus;

   /* Variable to indicate IOs which have fault */
   uint16 ioFaultStatus;

   /* Reserved for class-b status report */
   uint8 classBReserved[1u];
   uint8 errorstatuscrc;

   /* live key status - one bit per sensor */
   uint8 touchKeyStatus[5u];

   /* simple CRC8 of touchKeyStatus */
   uint8 touchKeyCRC;

   /* Latched key status - one bit per sensor */
   uint8 touchKeyLatchStatus[5u];

   /* Simple CRC8 of touchKeyLatchStatus */
   uint8 touchKeyLatchStatusCRC;

   /* This register shows the number of key presses */
   uint16 keyPressCounter;

   /* Variable to indicate the centroid position of linear slider */
   uint8 linearSliderCentroid[4u];

   /* Variable to indicate the centroid position of radial slider */
   uint8 radialSliderCentroid[1u];

   /* Variable to indicate the last valid centroid position of linear slider */
   uint8 linearSliderCentroidLastPos[4u];

   /* Variable to indicate the last centroid position of radial slider */
   uint8 radialSliderCentroidLastPos[1u];

   /* This register shows the heart beat counter value */
   uint16 heartBeatCounterValue;

   /* Indicates the last reset reason */
   uint8 lastResetReason;

   /* Reserved byte to ensure that touch delta is 2-byte aligned */
   uint8 deviceCurrentState;
}TOUCHI2C_STATUS_REGS;

typedef struct {
	uint8 numberOfSlaves;
	uint8 numberOfKeys;
	uint8 slaveAddress[TOUCHI2C_TOTAL_NUMBER_OF_SLAVE_DEVICES];
	uint8 keysPerSlave[TOUCHI2C_TOTAL_NUMBER_OF_SLAVE_DEVICES];
	uint8 slaveAddressToBusMap[TOUCHI2C_TOTAL_NUMBER_OF_SLAVE_DEVICES];
	uint8 keysToSlaveKeyMapping[TOUCHI2C_TOTAL_NUMBER_OF_KEYS];
} DEDICATED_TOUCH_INFO;

typedef struct
{
    unsigned char Offset_LSB;
    unsigned char I2C_Command;
    unsigned char Command_CRC;
    unsigned char Command_Counter;
}TOUCHI2C_COMMAND_TYPEUPDATE;

//! ReadWidget enable LPM MODE buffer data  from  Cypress Device
typedef  struct
{
    //! pointer to read or write memory location
    unsigned char  Widget_enable_Lp_Mode_Buffer[4];

}WIDGET_ENABLE_LPMODE_BUFFER_INFO;

typedef enum
{
    CYPRESSTOUCH_INIT=0,
    CYPRESSTOUCH_CURRENTSTATE_LOW_POWER_MODE_STATE = 1,
    CYPRESSTOUCH_CURRENTSTATE_ACTIVE_MODE_STATE = 2,
    CYPRESSTOUCH_CURRENTSTATE_INVALID=0xff

}CYPRESS_TOUCH_CURRENT_STATE;

typedef enum
{
    CYPRESS_REQUEST_TO_ACTIVE_MODE = 0,
    CYPRESS_REQUEST_TO_LOW_POWER_MODE = 1,
    CYPRESS_REQUEST_MODE_COMPLETE=2,
    CYPRESS_REQUEST_MODE_INVALID=0xff
}CYPRESS_TOUCH_POWERMODE_REQUEST_STATE;

void TouchI2c__Initialize(void);
void TouchI2c__FastHandler(void);
void TouchI2c__SlowHandler(void);

unsigned char TouchI2c__GetCommandErrorStatus(unsigned char slave_index);
T_BOOL TouchI2c__GetProjectFWType(void);
unsigned char TouchI2c__GetLastDeviceErrorCounter(unsigned char slave_index);
unsigned char TouchI2c__GetLastDeviceErrorStatus(unsigned char slave_index);
unsigned int TouchI2c__GetHangupCounter(unsigned char slave_index);
uint8* TouchI2c__GetKeyStatus(void);
uint16 TouchI2c__GetClassBErrorStatus(uint8 slave_index);
PASS_FAIL_TYPE TouchI2c__ReadRequestForSlaveDevice(TOUCHI2C_COMMAND_STRUCT *readRequestInfo,unsigned char device_index);
PASS_FAIL_TYPE TouchI2c__WriteRequestForSlaveDevice(TOUCHI2C_COMMAND_STRUCT *writeRequestInfo,unsigned char device_index);
int TouchI2c__GetDeviceReadWriteStatus(unsigned char touchI2cSlaveDeviceIndex);
int TouchI2c__GetDeviceReadWriteCmdRequestStatus(unsigned char touchI2cSlaveDeviceIndex);
int TouchI2c__Scanning(void);
T_BOOL TouchI2c__GetStatusReportInfo(void);
void TouchI2c__SetStatusReportInfo(T_BOOL status_info);
void TouchI2c__ResetDevice(unsigned char I2cDevice);
T_BOOL TouchI2C__SendI2CCommand(TOUCHI2C_PROTOCOL_COMMANDS i2cCommand, unsigned char SlaveIndex);
DEDICATED_TOUCH_INFO *TouchI2C__GetCYHardwareInfo(void);
TOUCHI2C_STATUS_REGS *TouchI2C__GetStatusReport(unsigned char slaveIndex);
unsigned int TouchI2C__GetTouchI2CHeartbeatCounter(void);
unsigned char TouchI2C__GetSlaveIndexForChannel(unsigned char channelNum);
unsigned char TouchI2C__GetI2cDeviceIndex(unsigned char i2c_busid);
#if (LOW_POWER_MODE_FEATURE==ENABLED)
PASS_FAIL_TYPE TouchI2C__GetWidgetEnableBufferRequestInfo(unsigned char deviceindex);
WIDGET_ENABLE_LPMODE_BUFFER_INFO  *TouchI2C_GetWidgetEnableLPMModeBufferData(unsigned char slaveIndex);
BOOL_TYPE TouchI2c__SetLowPowerRequestState(CYPRESS_TOUCH_POWERMODE_REQUEST_STATE set_request_low_power_mode); // ENUM 0,1,2 for set_value and Enter_power_mode
CYPRESS_TOUCH_CURRENT_STATE TouchI2c__GetTouchPowerStatus(void);
#endif
#endif  //#if(TOUCH_I2C_PROTOCOL == ENABLED)

#endif  //#define SOURCE_APPSPECIFIC_DRIVER_TOUCHI2C_TOUCHI2C_H_
