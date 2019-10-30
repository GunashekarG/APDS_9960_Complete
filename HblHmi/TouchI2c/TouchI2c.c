/*
 * TouchI2c.c
 *
 *  Created on: Nov 29, 2016
 *      Author: bhimahv
 */


/*  @copyright  Copyright 2011-. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//-------------------------------------- Include Files ----------------------------------------------------------------

#include "SystemConfig.h"

#if(TOUCH_I2C_PROTOCOL == ENABLED)
#include "I2c.h"
#include "Gpio.h"
#include "TouchI2c.h"
#include "TouchI2c_prv.h"
#include "Crc8.h"
#include "Utilities.h"
#ifndef GESE_SETTING_TOUCH_PARAMETERS
        #define GESE_SETTING_TOUCH_PARAMETERS  DISABLED
#endif //This macro closing for GESE_SETTING_TOUCH_PARAMETERS

#if (GESE_SETTING_TOUCH_PARAMETERS == ENABLED)
#include "SettingFile.h"
#endif
#include "API018Diagnostic.h"
//-------------------------------------- PRIVATE (Constants & Defines) -------------------------------------
//! Defines the maximum I2C command buffer for each slave device
//! First buffer is always used for continuous reading key status information from slave device
//! Second buffer is used for reading from//writing to slave device
#define TOUCHI2C_MAX_CMD_BUFFER_PER_SLAVE     	    2
#define TOUCHI2C_ONE_SECOND_TO_FIVE_MS_RATIO       (200)

//! KEY BUFFER SIZE
#define TOUCH_KEY_BUFFER_SIZE   5
//! Baseline reset time out of comamnd is 400 *5msec =2000msec
#define BASELINE_RESET_TIMEOUT 400
#if TOUCHI2C_SLIDER_ENABLED
#define TOUCHI2C_NUMBER_OF_STATUS_BYTES_TO_READ     10
#else
#define TOUCHI2C_NUMBER_OF_STATUS_BYTES_TO_READ     8
#endif

//---------------------------------------LPM MODE debug variables-----------------------------------------------------------------------------
//!ACTIVE TO  LOW power MODE STATES
#if (LOW_POWER_MODE_FEATURE == ENABLED)
#define RESET_I2C_NONE          (0xFF)
#define RESET_I2C_TIME_MS       (DELAY_COUNT_RESET_SLAVE/5)  // reset line handled at 5ms
#define SEND_ACTIVE_RETRIES     (3)     //how many times to send the command
 typedef enum
 {
     STATE_ENTER_LOW_POWER_IDLE = 0,
     STATE_ENTER_LOW_POWER_SEND_CMD,
     STATE_ENTER_LOW_POWER_STATUS_CHECK,
     STATE_ENTER_LOW_POWER_NEXT_DEVICE,
     STATE_ENTER_LOW_POWER_DONE,
     STATE_ENTER_LOW_POWER_INVALID=0xFF
 }TOUCHI2C_ENTER_LPM_STATE_TYPE;


 //!LPM to ACTIVE MODE STATES
 typedef enum
 {
     STATE_ENTER_ACTIVE_IDLE = 0,
     STATE_ENTER_ACTIVE_SEND_CMD,
     STATE_ENTER_ACTIVE_STATUS_CHECK,
     STATE_ENTER_ACTIVE_NEXT_DEVICE,
     STATE_ENTER_ACTIVE_DONE,
     STATE_ENTER_ACTIVE_BUS_RESET,
     STATE_ENTER_ACTIVE_INVALID=0xff,
 }TOUCHI2C_ENTER_ACTIVEMODE_STATE_TYPE;


 //!Reading Low Power mode buffere from Cypress & set state as per Device request.
 typedef enum
 {
     DEVICE_IDLE = 0,
     DEVICE_MODE_REQUEST=1,
     DEVICE_COMPLETE_RESPONSE=2,
     DEVICE_NO_UPDATE=0xFF
 }DEVICE_LP_STATUSBUFFER_MODE;


 typedef enum
 {
     CYPRESSTOUCH_MODE_INPROGRESS = 0,
     CYPRESSTOUCH_MODE_LOW_POWER_STATE=1,
     CYPRESSTOUCH_MODE_ACTIVE_STATE=2,
     CYPRESSTOUCH_MODE_INVALID_STATE=0xFF
 }CYPRESS_TOUCH_POWER_MODE_STATUS;

 DEVICE_LP_STATUSBUFFER_MODE Device_LP_Status_Buffer=DEVICE_IDLE;


 TOUCHI2C_ENTER_LPM_STATE_TYPE Low_power_Mode_State=STATE_ENTER_LOW_POWER_IDLE;

 TOUCHI2C_ENTER_ACTIVEMODE_STATE_TYPE Active_Mode_State=STATE_ENTER_ACTIVE_IDLE;

 CYPRESS_TOUCH_CURRENT_STATE  Cypress_Touch_Current_State;

 uint32 WidgetEnable_LPM_Buffer=0;
static uint8 Bkup_Lpm_time_Value;
uint8 Lpm_Tick_time;
static CYPRESS_TOUCH_POWER_MODE_STATUS Cypress_TouchPowerMode_Status[2];
static CYPRESS_TOUCH_POWERMODE_REQUEST_STATE Cypress_TouchPowerMode_Request_State;
//! ReadWidget enable LPM MODE buffer data  from  Cypress Device
WIDGET_ENABLE_LPMODE_BUFFER_INFO Widget_Enable_Buffer_Info[TOUCHI2C_TOTAL_NUMBER_OF_SLAVE_DEVICES];//TODO

static uint8 Reset_I2c_Devices;
static uint8 Send_Active_Cmd_Retry=0;
#endif
//-------------------------------------- Public (Constants & Defines) -------------------------------------

//! Address from slave device where Key status is read
#define TOUCHI2C_KEY_STATUS_SLAVE_OFFSET_MSB		0X00
#define TOUCHI2C_KEY_STATUS_SLAVE_OFFSET_LSB		0X0E

#define TOUCHI2C_STATUS_REPORT_SLAVE_OFFSET_MSB		0X00
#define TOUCHI2C_STATUS_REPORT_SLAVE_OFFSET_LSB		0X00

//! This is used to delay first I2C transaction  after device boot up
//! Delay in seconds = TOUCHI2C_STARTUP_DELAY_TIMEOUT * rate at which I2C handler is called
#ifndef TOUCHI2C_STARTUP_DELAY_TIMEOUT
#define TOUCHI2C_STARTUP_DELAY_TIMEOUT  			1000
#endif

//! Time interval to validate heart beat from slave devices
//! Delay in seconds = TOUCHI2C_HEARBEAT_CHECK_TIMEOUT * rate at which I2C handler is called
#ifndef TOUCHI2C_HEARBEAT_CHECK_TIMEOUT
#define TOUCHI2C_HEARBEAT_CHECK_TIMEOUT 			200
#endif

//! If device is not responding or in busy state wait until this delay before initializing I2C Bus
//! Delay in seconds = TOUCHI2C_HANGUP_DELAY_TIMEOUT * rate at which I2C handler is called
#ifndef TOUCHI2C_HANGUP_DELAY_TIMEOUT
#define TOUCHI2C_HANGUP_DELAY_TIMEOUT   			100
#endif

//! If Device is not responding i.e. Heart beat is not incrementing then wait until debounce counter before reseting I2C BUS
#ifndef TOUCHI2C_HEARTBEAT_FAIL_DEBOUNCE_COUNTER
#define TOUCHI2C_HEARTBEAT_FAIL_DEBOUNCE_COUNTER    5
#endif

//! Heart beat Error code
#ifndef TOUCHI2C_HEARTBEAT_FAIL
#define TOUCHI2C_HEARTBEAT_FAIL                     1
#endif

//! If device is reset due to heart beat not being received, generate a keypad disconnected fault after this number of resets.
#ifndef TOUCHI2C_KEYPAD_DISCONNECTED_DEBOUNCE
    #define TOUCHI2C_KEYPAD_DISCONNECTED_DEBOUNCE   3
#endif

//! Constant stores information related to the touch HMI
//! Values for the constant is used from TOUCHI2C_PRM_H
#if (GESE_SETTING_TOUCH_PARAMETERS != ENABLED)
const DEDICATED_TOUCH_INFO Cypress_Hardware_info =
{
    TOUCHI2C_TOTAL_NUMBER_OF_SLAVE_DEVICES,
    TOUCHI2C_TOTAL_NUMBER_OF_KEYS,
    TOUCHI2C_SLAVE_ADDR_LIST,
    TOUCHI2C_NUMBER_KEYS_PER_EACH_DEVICE,
    TOUCHI2C_SLAVE_DEVICE_LIST,
    TOUCHI2C_CHANNELS_AS_PER_HARDWARE_MAP_LIST,
};
#else
static DEDICATED_TOUCH_INFO Cypress_Hardware_info;
#endif

//-------------------------------------- PRIVATE (Global static variables) ------------------------------------------------
static unsigned char TOUCHI2C_SlaveIndex_To_KeyNumRange[TOUCHI2C_TOTAL_NUMBER_OF_SLAVE_DEVICES];

//! Structure to store the data read from key status register map
static TOUCHI2C_STATUS_REGS TouchI2c_Device_Status[TOUCHI2C_TOTAL_NUMBER_OF_SLAVE_DEVICES];

//! Structure to issue I2C command from I2C master to slave device
static TOUCHI2C_COMMAND_STRUCT TouchI2c_StateDeviceInfo[TOUCHI2C_TOTAL_NUMBER_OF_SLAVE_DEVICES][TOUCHI2C_MAX_CMD_BUFFER_PER_SLAVE];

//! Delay during startup before initializing I2C
static unsigned int TouchI2c_Startup_delay_timeout = TOUCHI2C_STARTUP_DELAY_TIMEOUT;

//! Variable to keep track of slave device index value for which I2C transaction has to be initiated
//! for each device whenever the API is called
static unsigned char TouchI2c_Slave_Device_Index = 0;

//! Variable to keep track of command index i.e whether to read key status or send custom I2C command
static unsigned char TouchI2c_Slave_Device_Command_Index = 0;

//! Variable to hold the number of tries for which slave has not responded
static unsigned int TouchI2c_Hangup_Counter[TOUCHI2C_TOTAL_NUMBER_OF_SLAVE_DEVICES];

//! Variable to hold the number of read for which heart beat value is not changing
static unsigned int TouchI2c_HeartBeat_Slave_Counter[TOUCHI2C_TOTAL_NUMBER_OF_SLAVE_DEVICES];

//! Variable to indicate when to validate heart beat
static unsigned int TouchI2c_Heartbeat_delay_timeout = TOUCHI2C_HEARBEAT_CHECK_TIMEOUT;

//! If heart beat is not valid for debounce samples, device will be reset
static unsigned int TouchI2c_HeartBeat_Fail_Debounce_Counter[TOUCHI2C_TOTAL_NUMBER_OF_SLAVE_DEVICES];

//! Allocate memory to store the key status data for the HMI
static unsigned char TouchI2C_KeyStatus[TOUCH_KEY_BUFFER_SIZE] = {0,0,0,0,0};

//! Counter is increment every 5ms. When counter reaches the desired value, status report data will be read
static unsigned int TouchI2C_StatusReport_Read_Counter = 0;

//!Counter is increment every 5msec . When counter reaches the desired value Keystatus update
static uint8 TouchI2c_KeystatusUpdate_Counter;

//! Variable incremented every time when application calls TouchI2c__Scanning function.
//! This will help touch commander know whether I2C communication is initiated by main application
static unsigned int TouchI2C_Heartbeat_Counter = 0;

//! Flag to indicate that status report data has to be read
static T_BOOL TouchI2C_StatusReport_Read_Flag = FALSE;

//! Current Device state update as per status Register & update this into API17
static T_BOOL Status_Report_info=FALSE;

//! Flag to check whether status report data requested is initiated or not
static T_BOOL TouchI2C_StatusReport_Is_Read_Initiated = FALSE;

//! verify  this Flag  is reset Baseline command write properly  during keystuck function
static unsigned char SenderBase_Linecommand;

//! Global parameter used for  Slave index number with respect to slaves
static unsigned char Baseline_Slaveindex;

//! This structure contain offset,crc, command counter members used for send reset baseline command
TOUCHI2C_COMMAND_TYPEUPDATE TouchI2C_Command_Update;
//! If Key stuck fail counter  array with respect to number of keys
static unsigned int TouchI2c_KeyStuck_Fail_Counter[TOUCHI2C_TOTAL_NUMBER_OF_KEYS];

//! If commnad TOUCHI2C_CMD_RESET_SENSOR_BSLN not send properly then we added retry timeout counter
unsigned short int  BaselineReset_Timeout;

//! Max duration set for  keys long press or stuck with respect to slave i.e. {3} 3 second  for slave 0   device  keys 
unsigned char  TouchI2c_FailurerecoverInterval[TOUCHI2C_TOTAL_NUMBER_OF_SLAVE_DEVICES] = TOUCHI2C_KEYS_STUCK_INTERVAL;

//! Device reset counter. A keypad disconnected fault is generated when it reaches TOUCHI2C_KEYPAD_DISCONNECTED_DEBOUNCE.
static uint8 TouchI2c_HeartBeat_Reset_Counter[TOUCHI2C_TOTAL_NUMBER_OF_SLAVE_DEVICES];

unsigned char key_stuck_counter_index = 0;
static uint8 Low_power_Device_index;
static uint32 Low_Power_SensorNumber;
static uint8 LowPower_Current_Device_Index;
#if (GESE_SETTING_TOUCH_PARAMETERS == ENABLED)
static uint8 Low_Power_Temp[3];
#endif


//-------------------------------------- PRIVATE (Prototype function ) ------------------------------------------------
static TOUCHI2C_HW_STATE TouchI2c_StateMachine(TOUCHI2C_COMMAND_STRUCT *TouchI2c_Datastruct);
static void TouchI2c_HearBeatValidation(void);
static void TouchI2c_FailureCallback(unsigned char error_type, unsigned char slave_id, unsigned int hearbeat_last_counter);
void TouchI2C_Update_I2C_Cmd_Buffer_SensorStatus(T_BOOL Read_SensorStatus);
static uint8 SystemTimer_GetTimeElapsed(uint8 currentTimeStamp, uint8 prevTimeStamp);
#if (LOW_POWER_MODE_FEATURE==ENABLED)
static CYPRESS_TOUCH_POWER_MODE_STATUS GetPsoCDeviceMode(uint8 deviceIndex);
static BOOL_TYPE TouchI2c_DeviceEnterLowPowerMode(void);
static BOOL_TYPE TouchI2c_DeviceEnterActiveMode(void);
#endif
static void TouchI2C_KeystuckRecoverHandler(void);
static void  TouchI2cUpdateStatus(void);
//=====================================================================================================================
//-------------------------------------- Public Functions -------------------------------------------------------------
//=====================================================================================================================

void TouchI2c__Initialize(void)
{
    unsigned char I2cDevice;
Low_power_Device_index=0;
#if (GESE_SETTING_TOUCH_PARAMETERS == ENABLED)
    SETTINGFILE_LOADER_TYPE aux_loader;
    uint8 index=0;

    Low_Power_SensorNumber=0;


    if (SettingFile__BasicLoader(SF_PTR_UI_TOUCH_PARAMETERS, 0, &aux_loader) == PASS )

    {
        Cypress_Hardware_info.numberOfSlaves = aux_loader.Data[0];

        Cypress_Hardware_info.numberOfKeys = 0;
        for(index = 0; index < Cypress_Hardware_info.numberOfSlaves; index++)
        {
            Cypress_Hardware_info.slaveAddressToBusMap[index] = (uint8)((aux_loader.Data[2 + index * 2] & 0xF0) >> 4);
            Cypress_Hardware_info.slaveAddress[index] = (uint8)aux_loader.Data[2 + index * 2] & 0x0F;
            Cypress_Hardware_info.keysPerSlave[index] = aux_loader.Data[3 + index * 2];
            Cypress_Hardware_info.numberOfKeys += aux_loader.Data[3 + index * 2]; //TODO VJ Read from GESE
        }
        for(index = 0; index < Cypress_Hardware_info.numberOfKeys; index++)
        {
            Cypress_Hardware_info.keysToSlaveKeyMapping[index] = aux_loader.Data[2 +( Cypress_Hardware_info.numberOfSlaves * 2) + index];
        }

        index=0;
        index= (2 +( Cypress_Hardware_info.numberOfSlaves * 2)+  Cypress_Hardware_info.numberOfKeys);

        index++;
        Low_power_Device_index=aux_loader.Data[index];
        index++;
        Low_Power_Temp[0]=aux_loader.Data[index];  //MSB
        index++;
        Low_Power_Temp[1]=aux_loader.Data[index];
        index++;
        Low_Power_Temp[2]=aux_loader.Data[index];//LSB

        index=0;
        Low_Power_SensorNumber=0;
        Low_Power_SensorNumber=Utilities__ConvertArrayTo24bits(Low_Power_Temp);
    }


 #endif

#if ((GESE_SETTING_TOUCH_PARAMETERS != ENABLED)&&(LOW_POWER_MODE_FEATURE==ENABLED))
   // Low_power_Device_index =LOW_POWER_MODE_DEVICE_NO;
        Low_power_Device_index= LOW_POWER_MODE_DEVICE_NO;
        Low_Power_SensorNumber=LOW_POWER_ENABLED_SENSOR_BUTTON_NUMBER;
#endif

    SenderBase_Linecommand=0;
    Baseline_Slaveindex=0;
    BaselineReset_Timeout=0;
    key_stuck_counter_index = 0;
    TouchI2c_Slave_Device_Index = 0;
    TouchI2c_Slave_Device_Command_Index = 0;
    TouchI2c_Startup_delay_timeout = TOUCHI2C_STARTUP_DELAY_TIMEOUT;   //Reload Time  delay for first I2C transaction  after device boot up
    TouchI2c_Heartbeat_delay_timeout = TOUCHI2C_HEARBEAT_CHECK_TIMEOUT; //Reload Time interval to validate  Heart beat from slave devices.

   for(key_stuck_counter_index=0;key_stuck_counter_index<5;key_stuck_counter_index++)
   {

       TouchI2C_KeyStatus[key_stuck_counter_index] = 0x00;
   }

   key_stuck_counter_index=0;

    for(I2cDevice = 0; I2cDevice < Cypress_Hardware_info.numberOfSlaves; I2cDevice++)
    {
        //! Initialize I2C Device parameters

        //! TODO - Function is empty. Please check
//        TouchI2c__ResetDevice(I2cDevice);

        I2c__Clear(Cypress_Hardware_info.slaveAddressToBusMap[I2cDevice]);

        //! Re-init I2C bus for next transaction
      I2c__Initialize(Cypress_Hardware_info.slaveAddressToBusMap[I2cDevice], TOUCH_I2C_SPEED, I2C_ADDR_7BITS,Cypress_Hardware_info.slaveAddress[I2cDevice]);

        TouchI2c_Hangup_Counter[I2cDevice] = 0;
        TouchI2c_HeartBeat_Slave_Counter[I2cDevice] = 0;
        TouchI2c_HeartBeat_Fail_Debounce_Counter[I2cDevice] = 0;

		//! Load the I2C command buffer to read key status continuously
		TouchI2c_StateDeviceInfo[I2cDevice][0].TouchI2c_Slave_Device = Cypress_Hardware_info.slaveAddressToBusMap[I2cDevice]; //TouchI2c_Slave_Device[I2cDevice];
		TouchI2c_StateDeviceInfo[I2cDevice][0].slaveAddress = (Cypress_Hardware_info.slaveAddress[I2cDevice] << 1); //TouchI2c_Slave_Addr[I2cDevice];

		//TODO: Need to fix the start address
		TouchI2c_StateDeviceInfo[I2cDevice][0].slaveOffsetMSB = TOUCHI2C_KEY_STATUS_SLAVE_OFFSET_MSB;
		TouchI2c_StateDeviceInfo[I2cDevice][0].slaveOffsetLSB = TOUCHI2C_KEY_STATUS_SLAVE_OFFSET_LSB;
		TouchI2c_StateDeviceInfo[I2cDevice][0].i2c_read_write= TOUCHI2C_READ_COMMAND;

		//TODO: Need to fix the number of bytes based on whether slider is enabled
		TouchI2c_StateDeviceInfo[I2cDevice][0].number_Of_read_write_bytes= 12;//sizeof(TouchI2c_Device_Status[I2cDevice]);

		//TODO: Need to change the starting address depending on whether slider is enabled
		TouchI2c_StateDeviceInfo[I2cDevice][0].src_dst_buffer_ptr = (unsigned char*) &TouchI2c_Device_Status[I2cDevice].touchKeyStatus[0];

		TouchI2c_StateDeviceInfo[I2cDevice][0].touchI2C_Hw_State = TOUCHI2C_SUCCESS;  //TouchI2c low level state initialize

		//TODO: Need to fix below code
		TouchI2c_StateDeviceInfo[I2cDevice][0].Send_Command_Continously = 1; //Always read key status
		TouchI2c_StateDeviceInfo[I2cDevice][0].IscmdRequest = TRUE;
		TouchI2c_StateDeviceInfo[I2cDevice][0].TouchI2c_Hangup_Delay_Timeout = TOUCHI2C_HANGUP_DELAY_TIMEOUT;
		TouchI2c_StateDeviceInfo[I2cDevice][0].TouchI2c_FM_State = TOUCHI2C_STATE_INIT;  //TouchI2C Application state initialize

		if (I2cDevice > 0) {
			TOUCHI2C_SlaveIndex_To_KeyNumRange[I2cDevice] = TOUCHI2C_SlaveIndex_To_KeyNumRange[I2cDevice-1] + Cypress_Hardware_info.keysPerSlave[I2cDevice];
		}
		else
		{
			TOUCHI2C_SlaveIndex_To_KeyNumRange[I2cDevice] = Cypress_Hardware_info.keysPerSlave[I2cDevice];
		}

		TouchI2c_HeartBeat_Reset_Counter[I2cDevice]=0;
    }

    for(uint8 keyindex=0;keyindex<Cypress_Hardware_info.numberOfKeys;keyindex++)
    {
        TouchI2c_KeyStuck_Fail_Counter[keyindex]=0;
    }

#if (LOW_POWER_MODE_FEATURE==ENABLED)
    Low_power_Mode_State=STATE_ENTER_LOW_POWER_IDLE;
    Active_Mode_State=STATE_ENTER_ACTIVE_IDLE;
    WidgetEnable_LPM_Buffer=0;
    Bkup_Lpm_time_Value=0;
    Lpm_Tick_time=0;
    Device_LP_Status_Buffer=DEVICE_IDLE;
    //! @ reboot or startup Cypress Touch request into Active Mode
    Cypress_TouchPowerMode_Request_State=CYPRESS_REQUEST_TO_ACTIVE_MODE;
    Cypress_TouchPowerMode_Status[0] =CYPRESSTOUCH_MODE_INPROGRESS;
    Cypress_TouchPowerMode_Status[1] =CYPRESSTOUCH_MODE_INPROGRESS;
    Cypress_Touch_Current_State=CYPRESSTOUCH_INIT;
    Reset_I2c_Devices = RESET_I2C_NONE;
    Send_Active_Cmd_Retry = 0;

#endif

    TouchI2c_KeystatusUpdate_Counter=0;
}

//! This function can be called from application which return the status whether it is idle or busy. This will help application to perform any other action when
//! Touch I2c module return idle. When application call this function...then they never should call TouchI2c__FastHanlder.
int TouchI2c__Scanning(void)
{
	TOUCHI2C_HW_STATE ret;

	//! Wait until  delay for first I2C transaction  after device boot up
    if (TouchI2c_Startup_delay_timeout)
    {
        TouchI2c_Startup_delay_timeout--;
        return TOUCHI2C_BUSY;
    }

    //! Increment the counter every time when Touch I2C is called
    //! This data will be used by touch commander to know if application is initiating
    //! I2C communication
    TouchI2C_Heartbeat_Counter++;

    //! If a command is requested, initiate I2C transaction
    if (TouchI2c_StateDeviceInfo[TouchI2c_Slave_Device_Index][TouchI2c_Slave_Device_Command_Index].IscmdRequest)
    {
        //! Initiate I2C transaction
        ret = TouchI2c_StateMachine(&TouchI2c_StateDeviceInfo[TouchI2c_Slave_Device_Index][TouchI2c_Slave_Device_Command_Index]);

        //! If command is successfully executed or an I2C error occurs, move to the next command request
        if ((ret == TOUCHI2C_SUCCESS) || (ret == TOUCHI2C_ERROR))
        {
        	//! In case, command is not requested to be send periodically, reset the command request bit
            if (!TouchI2c_StateDeviceInfo[TouchI2c_Slave_Device_Index][TouchI2c_Slave_Device_Command_Index].Send_Command_Continously)
            {
                TouchI2c_StateDeviceInfo[TouchI2c_Slave_Device_Index][TouchI2c_Slave_Device_Command_Index].IscmdRequest = 0;
            }

            //! Move to next command
            TouchI2c_Slave_Device_Command_Index++;

            //! If all command is serviced for current device, move to next device
            if (TouchI2c_Slave_Device_Command_Index >= (TOUCHI2C_MAX_CMD_BUFFER_PER_SLAVE))
            {
                TouchI2c_Slave_Device_Command_Index = 0;
                TouchI2c_Slave_Device_Index++;

                //! If all devices are serviced, reset the device index
                if (TouchI2c_Slave_Device_Index >= (Cypress_Hardware_info.numberOfSlaves))//TOUCHI2C_TOTAL_NUMBER_OF_SLAVE_DEVICES
                {
                	TouchI2c_Slave_Device_Index = 0;

                	//! Check if entire status report area has to be read
                	if(TouchI2C_StatusReport_Read_Flag == TRUE)
                	{
                		//! If entire status report read is not requested yet, initiate entire area read
                		if(TouchI2C_StatusReport_Is_Read_Initiated == FALSE)
                		{
							TouchI2C_Update_I2C_Cmd_Buffer_SensorStatus(TRUE);
							TouchI2C_StatusReport_Is_Read_Initiated = TRUE;
                		}
                		//! If entire status report is read, revert back to read only selected data
                		else
                		{
							TouchI2C_Update_I2C_Cmd_Buffer_SensorStatus(FALSE);
							TouchI2C_StatusReport_Is_Read_Initiated = FALSE;
							TouchI2C_StatusReport_Read_Flag = FALSE;
							Status_Report_info=TRUE;
                		}
                	}
                	return TOUCHI2C_IDLE;
                }
            }
        }
        /* If the current slave is busy, move to the next slave */
        else if ((ret == TOUCHI2C_BUSY) || (ret == TOUCHI2C_TIMEOUT))
        {
        	if((ret == TOUCHI2C_TIMEOUT))
        	{
				//! In case, command is not requested to be send periodically, reset the command request bit
				if (!TouchI2c_StateDeviceInfo[TouchI2c_Slave_Device_Index][1].Send_Command_Continously)
				{
					TouchI2c_StateDeviceInfo[TouchI2c_Slave_Device_Index][1].IscmdRequest = 0;
				}
        	}
        	TouchI2c_Slave_Device_Command_Index = 0;
			TouchI2c_Slave_Device_Index++;

			//! If all devices are serviced, reset the device index
            if (TouchI2c_Slave_Device_Index >= (Cypress_Hardware_info.numberOfSlaves))
			{
				TouchI2c_Slave_Device_Index = 0;

				//! Check if entire status report area has to be read
				if(TouchI2C_StatusReport_Read_Flag == TRUE)
				{
					//! If entire status report read is not requested yet, initiate entire area read
					if(TouchI2C_StatusReport_Is_Read_Initiated == FALSE)
					{
						TouchI2C_Update_I2C_Cmd_Buffer_SensorStatus(TRUE);
						TouchI2C_StatusReport_Is_Read_Initiated = TRUE;
					}
					//! If entire status report is read, revert back to read only selected data
					else
					{
						TouchI2C_Update_I2C_Cmd_Buffer_SensorStatus(FALSE);
						TouchI2C_StatusReport_Is_Read_Initiated = FALSE;
						TouchI2C_StatusReport_Read_Flag = FALSE;
						Status_Report_info=TRUE;
					}
				}
			}
			return TOUCHI2C_IDLE;
        }
    }
    //! If command is not requested, move to the next command request
    else
    {
    	//! Move to next command
    	TouchI2c_Slave_Device_Command_Index++;

    	//! If all command is serviced for current device, move to next device
        if (TouchI2c_Slave_Device_Command_Index >= (TOUCHI2C_MAX_CMD_BUFFER_PER_SLAVE))
        {
        	TouchI2c_Slave_Device_Command_Index = 0;
        	TouchI2c_Slave_Device_Index++;

        	//! If all devices are serviced, reset the device index
            if (TouchI2c_Slave_Device_Index >= (Cypress_Hardware_info.numberOfSlaves))//TOUCHI2C_TOTAL_NUMBER_OF_SLAVE_DEVICES
            {
            	TouchI2c_Slave_Device_Index = 0;
            	//! Check if entire status report area has to be read
            	if(TouchI2C_StatusReport_Read_Flag == TRUE)
            	{
            		//! If entire status report read is not requested yet, initiate entire area read
            		if(TouchI2C_StatusReport_Is_Read_Initiated == FALSE)
            		{
						TouchI2C_Update_I2C_Cmd_Buffer_SensorStatus(TRUE);
						TouchI2C_StatusReport_Is_Read_Initiated = TRUE;
            		}
            		//! If entire status report is read, revert back to read only selected data
            		else
            		{
						TouchI2C_Update_I2C_Cmd_Buffer_SensorStatus(FALSE);
						TouchI2C_StatusReport_Is_Read_Initiated = FALSE;
						TouchI2C_StatusReport_Read_Flag = FALSE;
						Status_Report_info=TRUE;
            		}
            	}
            	return TOUCHI2C_IDLE;
            }
        }
    }
    return TOUCHI2C_BUSY;
}
//This function is used for Device status info update
T_BOOL TouchI2c__GetStatusReportInfo(void)
{
    return Status_Report_info;

}

void TouchI2c__SetStatusReportInfo(T_BOOL status_info)
{
     Status_Report_info= status_info;

}

/**
 * //! This function can be called from Main prv . THis will help application to perform any other action when
 *
 * This function return as void. If this function called from main Prv  don't call  TouchI2c__Scanning() from application. i.e. Hmi__fasthandler()
 */
//! Called at 250us or 1ms
void TouchI2c__FastHandler(void)
{
	TouchI2c__Scanning();
}

/**
 * @brief   This function handles the slow handler functionality
 * @param   void
 * @return  void
 */
//!Called at 5ms always
void TouchI2c__SlowHandler(void)
{
	TouchI2C_StatusReport_Read_Counter++;
	TouchI2c_KeystatusUpdate_Counter++;

	if(TouchI2c_KeystatusUpdate_Counter >=4)
	{
	    TouchI2c_KeystatusUpdate_Counter=0;
	    TouchI2cUpdateStatus();

	}

	if(TouchI2C_StatusReport_Read_Counter > (TOUCHI2C_STATUS_REPORT_READ_INTERVAL * TOUCHI2C_ONE_SECOND_TO_FIVE_MS_RATIO))
	{
		TouchI2C_StatusReport_Read_Counter = 0;
		TouchI2C_StatusReport_Read_Flag = TRUE;

	}
    TouchI2c_HearBeatValidation();
   TouchI2C_KeystuckRecoverHandler();




//This is used for Low power mode sensor debug tick time used
#if( LOW_POWER_MODE_FEATURE==ENABLED)
    Lpm_Tick_time++;

    // This piece of code will take care of entry mechanism  With respect to request state Low Power mode handle
    if(Cypress_TouchPowerMode_Request_State == CYPRESS_REQUEST_TO_ACTIVE_MODE)
    {
        if(TouchI2c_DeviceEnterActiveMode()==TRUE)
        {
            Cypress_TouchPowerMode_Request_State = CYPRESS_REQUEST_MODE_COMPLETE;
            Device_LP_Status_Buffer = DEVICE_IDLE;
            Active_Mode_State = STATE_ENTER_ACTIVE_IDLE;
            Cypress_Touch_Current_State = CYPRESSTOUCH_CURRENTSTATE_ACTIVE_MODE_STATE;
        }
    }
    else if(Cypress_TouchPowerMode_Request_State == CYPRESS_REQUEST_TO_LOW_POWER_MODE)
    {
        if(TouchI2c_DeviceEnterLowPowerMode() == TRUE)
        {
            Cypress_TouchPowerMode_Request_State = CYPRESS_REQUEST_MODE_COMPLETE;
            Device_LP_Status_Buffer = DEVICE_IDLE;
            Low_power_Mode_State = STATE_ENTER_LOW_POWER_IDLE;
            Cypress_Touch_Current_State = CYPRESSTOUCH_CURRENTSTATE_LOW_POWER_MODE_STATE;
        }
    }
    else
    {

    }

    if(Reset_I2c_Devices != RESET_I2C_NONE)
    {
        if(Reset_I2c_Devices)
        {
            Reset_I2c_Devices--;
            Gpio__PinConfig(TOUCHI2C_RESET_GPIO_PORT, TOUCHI2C_RESET_GPIO_PIN, OUTPUT_PUSHPULL);
            Gpio__PinWrite(TOUCHI2C_RESET_GPIO_PORT, TOUCHI2C_RESET_GPIO_PIN, FALSE);       // Put the Device into Reset.
        }
        else
        {
            Reset_I2c_Devices = RESET_I2C_NONE;
            Gpio__PinWrite(TOUCHI2C_RESET_GPIO_PORT, TOUCHI2C_RESET_GPIO_PIN, TRUE);        // release the Device from Reset.
        }
    }

#endif
}



/**
 * @brief   This function gets the key status  with respect to slave devices
 *
 * @return  uint8*: returns the key status  from updatekeystatus function
 */

uint8* TouchI2c__GetKeyStatus(void)
{
    return((uint8 *)TouchI2C_KeyStatus);

}

/**
 * @brief   This function gets hangup counter for given slave device
 * @param   unsigned char slave_index
 * @return  unsigned int hangup counter
 */

unsigned int TouchI2c__GetHangupCounter(unsigned char slave_index)
{
    if(slave_index < Cypress_Hardware_info.numberOfSlaves)
    {
        return TouchI2c_Hangup_Counter[slave_index];
    }
    return 0;
}

/**
 * @brief   This function gets last error from slave device
 * @param   unsigned char slave_index
 * @return  unsigned char device last error status
 */

unsigned char TouchI2c__GetLastDeviceErrorStatus(unsigned char slave_index)
{
    if(slave_index < Cypress_Hardware_info.numberOfSlaves)
    {
        return TouchI2c_Device_Status[slave_index].errorStatus;
    }
    return 0;
}

/**
 * @brief   This function gets number of errors from slave device
 * @param   unsigned char slave_index
 * @return  unsigned char number of error in the device
 */

unsigned char TouchI2c__GetLastDeviceErrorCounter(unsigned char slave_index)
{
    if(slave_index < Cypress_Hardware_info.numberOfSlaves)
    {
        return TouchI2c_Device_Status[slave_index].errorCounter;
    }
    return 0;
}

/**
 * @brief   This function gets Command error status
 * @param   unsigned char slave_index
 * @return  unsigned char return command error status
 */

unsigned char TouchI2c__GetCommandErrorStatus(unsigned char slave_index)
{
    if(slave_index < Cypress_Hardware_info.numberOfSlaves)
    {
        return TouchI2c_Device_Status[slave_index].ctrlCmdError;
    }
    return 0;
}

/**
 * @brief   This function gets Command classBErrorStatus error status
 * @param   unsigned char slave_index
 * @return  unsigned char return command classBErrorStatus error status
 */

uint16 TouchI2c__GetClassBErrorStatus(uint8 slave_index)
{
    if(slave_index < Cypress_Hardware_info.numberOfSlaves)
    {
        return TouchI2c_Device_Status[slave_index].classBErrorStatus;
    }
    return 0;
}



/**
 * @brief   This function gets ClassA or ClassB project type  return 1 means CLASSB otherwise CLASSA
 * @param   unsigned char slave_index
 * @return  T_BOOL return ClassA or ClassB project type  SRTouchI2c__GetProjectFWType
 */

T_BOOL TouchI2c__GetProjectFWType(void)
 {
	T_BOOL ret;
	unsigned char slave_index;

	for(slave_index=0;slave_index < Cypress_Hardware_info.numberOfSlaves;slave_index++)
	{
		if ((TouchI2c_Device_Status[slave_index].deviceCurrentState)
				& (1 << 5))
		{
			ret = TRUE;  //! CLASSB FW TYPE
		}
		else
		{
			ret = FALSE;  //! CLASSA FW TYPE
		}
	}

	return ret;
}


PASS_FAIL_TYPE TouchI2c__ReadRequestForSlaveDevice(TOUCHI2C_COMMAND_STRUCT *readRequestInfo, unsigned char device_index)
{
	if(readRequestInfo == NULL)
	{
		return FAIL;
	}
	if(device_index >= Cypress_Hardware_info.numberOfSlaves)
	{
		return FAIL;
	}
	if(readRequestInfo->i2c_read_write != TOUCHI2C_READ_COMMAND)
	{
		return FAIL;
	}
	if(TouchI2c_StateDeviceInfo[device_index][1].TouchI2c_FM_State != TOUCHI2C_STATE_INIT)
	{
		return FAIL;
	}

	memcpy(&TouchI2c_StateDeviceInfo[device_index][1], readRequestInfo, sizeof(TOUCHI2C_COMMAND_STRUCT));
	return PASS;
}


PASS_FAIL_TYPE TouchI2c__WriteRequestForSlaveDevice(TOUCHI2C_COMMAND_STRUCT *writeRequestInfo, unsigned char device_index)
{


	if(writeRequestInfo == NULL)
	{
		return FAIL;
	}
	if(device_index >= Cypress_Hardware_info.numberOfSlaves)
	{
		return FAIL;
	}
	if(writeRequestInfo->i2c_read_write != TOUCHI2C_WRITE_COMMAND)
	{
		return FAIL;
	}
	//if(TouchI2c_StateDeviceInfo[writeRequestInfo->TouchI2c_Slave_Device][1].TouchI2c_FM_State != TOUCHI2C_STATE_INIT)
	    if(TouchI2c_StateDeviceInfo[device_index][1].TouchI2c_FM_State != TOUCHI2C_STATE_INIT)

	{
		return FAIL;
	}

	memcpy(&TouchI2c_StateDeviceInfo[device_index][1], writeRequestInfo, sizeof(TOUCHI2C_COMMAND_STRUCT));
	return PASS;
}

/**
 *    @brief    Returns the I2C finite machine state for a give slave index
 *
 *    @param    touchI2cSlaveDeviceIndex - index of slave for which state is requested
 *
 *    @return   I2C finite state machine state
 *
 */
int TouchI2c__GetDeviceReadWriteStatus(unsigned char touchI2cSlaveDeviceIndex)
{
	if(touchI2cSlaveDeviceIndex >= Cypress_Hardware_info.numberOfSlaves)
	{
		return -1;
	}

	return TouchI2c_StateDeviceInfo[touchI2cSlaveDeviceIndex][1].TouchI2c_FM_State;
}

/**
 *    @brief    Returns the status of command request register
 *
 *    @param    touchI2cSlaveDeviceIndex - index of slave for which command is requested
 *
 *    @return   command request status
 *
 */
int TouchI2c__GetDeviceReadWriteCmdRequestStatus(unsigned char touchI2cSlaveDeviceIndex)
{
	if(touchI2cSlaveDeviceIndex >= Cypress_Hardware_info.numberOfSlaves)
	{
		return -1;
	}

	return TouchI2c_StateDeviceInfo[touchI2cSlaveDeviceIndex][1].IscmdRequest;
}


/**
 *
 *
 * @param I2c_device
 */
void TouchI2c__ResetDevice(unsigned char I2cDevice)
{
#if ( TOUCHI2C_RESET_DEVICE == ENABLED )         // If to Reset the TouchI2c devices:   Gpio__PinConfig( (PORT_DEF) TOUCHI2C_RESET_GPIO_PORT, TOUCHI2C_RESET_GPIO_PIN, OUTPUT_PUSHPULL );         // Set Device Reset Port config

     Gpio__PinConfig(TOUCHI2C_RESET_GPIO_PORT, TOUCHI2C_RESET_GPIO_PIN, OUTPUT_PUSHPULL);
    Gpio__PinWrite(TOUCHI2C_RESET_GPIO_PORT, TOUCHI2C_RESET_GPIO_PIN, FALSE);       // Put the Device into Reset.

    //! Reset time should be  micro second
    Micro__DelayNumNops(DELAY_COUNT_RESET_SLAVE);

    Gpio__PinWrite( (PORT_DEF)  TOUCHI2C_RESET_GPIO_PORT, TOUCHI2C_RESET_GPIO_PIN, TRUE );   //Take the Device out of Reset.

#endif
}




/**
 *
 *@ Brief This function is used for send  I2C command  to slave device 
 *
 *@param i2cCommand,SlaveIndex
 */
T_BOOL TouchI2C__SendI2CCommand(TOUCHI2C_PROTOCOL_COMMANDS i2cCommand, unsigned char SlaveIndex)
{
    T_BOOL ret=FALSE;
    TOUCHI2C_COMMAND_STRUCT sensorconfigrequest;
    const DEDICATED_TOUCH_INFO *ptr2Cypress_Hw_info;
    ptr2Cypress_Hw_info =  TouchI2C__GetCYHardwareInfo();

//! After read/write request command is completed, send the status to touch commander
//! Check if command is executed
   // if (TouchI2c__GetDeviceReadWriteCmdRequestStatus(SRTouchI2C__GetI2cDeviceIndex(ptr2Cypress_Hw_info->slaveAddressToBusMap[SlaveIndex])) == 0)
    if (TouchI2c__GetDeviceReadWriteCmdRequestStatus(SlaveIndex) == 0)
    {

        TouchI2C_Command_Update.Offset_LSB = 0x00;
        TouchI2C_Command_Update.I2C_Command = (unsigned char)i2cCommand;
        TouchI2C_Command_Update.Command_Counter++;
        TouchI2C_Command_Update.Command_CRC = Crc8(0x00, &TouchI2C_Command_Update.I2C_Command, sizeof(TouchI2C_Command_Update.I2C_Command));

        //  API017Touch_CypressI2CData
        sensorconfigrequest.TouchI2c_Slave_Device = ptr2Cypress_Hw_info->slaveAddressToBusMap[SlaveIndex];
        sensorconfigrequest.slaveAddress =  (ptr2Cypress_Hw_info->slaveAddress[SlaveIndex]<<1);
        sensorconfigrequest.slaveOffsetMSB = 0x00;
        sensorconfigrequest.slaveOffsetLSB = 0x00;
        sensorconfigrequest.i2c_read_write = TOUCHI2C_WRITE_COMMAND;
        sensorconfigrequest.number_Of_read_write_bytes = sizeof(TOUCHI2C_COMMAND_TYPEUPDATE);
        sensorconfigrequest.src_dst_buffer_ptr  =  (unsigned char *)&TouchI2C_Command_Update.Offset_LSB;
        sensorconfigrequest.touchI2C_Hw_State = TOUCHI2C_SUCCESS;
        sensorconfigrequest.Send_Command_Continously = 0;
        sensorconfigrequest.IscmdRequest = 1;
        sensorconfigrequest.TouchI2c_Hangup_Delay_Timeout = 100;
        sensorconfigrequest.TouchI2c_FM_State = TOUCHI2C_STATE_INIT;

       // return TouchI2c__WriteRequestForSlaveDevice(&sensorconfigrequest, SRTouchI2C__GetI2cDeviceIndex(sensorconfigrequest.TouchI2c_Slave_Device));
        return TouchI2c__WriteRequestForSlaveDevice(&sensorconfigrequest,SlaveIndex);
    }

    return ret;
}

/**
 *
 *@ Brief This function is used for Baseline command. If Key Stuck for long duration then we reset that key
 *
 *@param i2cCommand,SlaveIndex,channel , Channel means logical number need to pass
 */
BOOL_TYPE TouchI2C__SendBaselineCommand(TOUCHI2C_PROTOCOL_COMMANDS i2cCommand, uint8 SlaveIndex,uint8 channel)
{
    BOOL_TYPE ret=FALSE;
    TOUCHI2C_COMMAND_STRUCT sensorconfigrequest;
    const DEDICATED_TOUCH_INFO *ptr2Cypress_Hw_info;
    ptr2Cypress_Hw_info =  TouchI2C__GetCYHardwareInfo();

//! After read/write request command is completed, send the status to touch commander
//! Check if command is executed
   // if (TouchI2c__GetDeviceReadWriteCmdRequestStatus(SRTouchI2C__GetI2cDeviceIndex(ptr2Cypress_Hw_info->slaveAddressToBusMap[SlaveIndex])) == 0)
    if (TouchI2c__GetDeviceReadWriteCmdRequestStatus(SlaveIndex) == 0)
    {

        TouchI2C_Command_Update.Offset_LSB = 0x00;
        TouchI2C_Command_Update.I2C_Command = (unsigned char)i2cCommand;
        TouchI2C_Command_Update.Command_Counter++;
        TouchI2C_Command_Update.Command_CRC = channel; //Crc8(0x00, &TouchI2C_Command_Update.I2C_Command, sizeof(TouchI2C_Command_Update.I2C_Command));

        //  API017Touch_CypressI2CData
        sensorconfigrequest.TouchI2c_Slave_Device = ptr2Cypress_Hw_info->slaveAddressToBusMap[SlaveIndex];
        sensorconfigrequest.slaveAddress =  (ptr2Cypress_Hw_info->slaveAddress[SlaveIndex]<<1);
        sensorconfigrequest.slaveOffsetMSB = 0x00;
        sensorconfigrequest.slaveOffsetLSB = 0x00;
        sensorconfigrequest.i2c_read_write = TOUCHI2C_WRITE_COMMAND;
        sensorconfigrequest.number_Of_read_write_bytes = sizeof(TOUCHI2C_COMMAND_TYPEUPDATE);
        sensorconfigrequest.src_dst_buffer_ptr  =  (unsigned char *)&TouchI2C_Command_Update.Offset_LSB;
        sensorconfigrequest.touchI2C_Hw_State = TOUCHI2C_SUCCESS;
        sensorconfigrequest.Send_Command_Continously = 0;
        sensorconfigrequest.IscmdRequest = 1;
        sensorconfigrequest.TouchI2c_Hangup_Delay_Timeout = 100;
        sensorconfigrequest.TouchI2c_FM_State = TOUCHI2C_STATE_INIT;

       // return TouchI2c__WriteRequestForSlaveDevice(&sensorconfigrequest, SRTouchI2C__GetI2cDeviceIndex(sensorconfigrequest.TouchI2c_Slave_Device));
        return TouchI2c__WriteRequestForSlaveDevice(&sensorconfigrequest,SlaveIndex );
    }

    return ret;
}
//=====================================================================================================================
//-------------------------------------- Private Functions -------------------------------------------------------------
//=====================================================================================================================
/**
 *    @brief    Initiates I2C read/write transaction for the command buffer
 *
 *    @param    I2C command buffer with information on I2C slave address, offset address and number of read/
 *              write bytes
 *
 *    @return   I2C transaction status
 *
 */
static TOUCHI2C_HW_STATE TouchI2c_StateMachine(TOUCHI2C_COMMAND_STRUCT *TouchI2c_Datastruct)
{
    unsigned short response;
    BOOL_TYPE ret;

    //! Get the current I2C state before initiating a transaction, Response can be I2C_STATE_BUSY or no busy
    response = I2c__GetStatus(TouchI2c_Datastruct->TouchI2c_Slave_Device, I2C_STATUS_STATE);

    //! If response is busy, increment hangup delay counter
    if (response == I2C_STATE_BUSY)
    {
        TouchI2c_Datastruct->TouchI2c_Hangup_Delay_Timeout++;

        //! If slave is busy for longer duration, reinitialize I2C communication to recover from any errors
        if (TouchI2c_Datastruct->TouchI2c_Hangup_Delay_Timeout >= TOUCHI2C_HANGUP_DELAY_TIMEOUT)
        {
            I2c__Clear(TouchI2c_Datastruct->TouchI2c_Slave_Device);

            //! Re-init I2C bus for next transaction
            response = I2c__Initialize(TouchI2c_Datastruct->TouchI2c_Slave_Device, TOUCH_I2C_SPEED, I2C_ADDR_7BITS,
            TouchI2c_Datastruct->slaveAddress);

            //! Re-init all the timeout variables
            TouchI2c_Datastruct->TouchI2c_Hangup_Delay_Timeout = 0;
            TouchI2c_Datastruct->TouchI2c_FM_State = TOUCHI2C_STATE_INIT;
            TouchI2c_Datastruct->touchI2C_Hw_State = TOUCHI2C_TIMEOUT;
            return TouchI2c_Datastruct->touchI2C_Hw_State;
        }
        //! I2C is busy, but timeout has not occurred. Initiate retry
        else
        {
            TouchI2c_Datastruct->touchI2C_Hw_State = TOUCHI2C_RETRY;
            return TOUCHI2C_RETRY;
        }
    }

    //! Get the current I2C status i.e. ACK/NACK is received
    response = I2c__GetStatus(TouchI2c_Datastruct->TouchI2c_Slave_Device, I2C_STATUS_ERROR);

    //! if an error is detected in I2C communication, re-init I2C BUS
    if (response != I2C_ERROR_NONE)
    {
        I2c__Clear(TouchI2c_Datastruct->TouchI2c_Slave_Device);

        //! Re-init I2C bus for next transaction
        response = I2c__Initialize(TouchI2c_Datastruct->TouchI2c_Slave_Device, TOUCH_I2C_SPEED, I2C_ADDR_7BITS,
        TouchI2c_Datastruct->slaveAddress);

        //! Re-init I2C state
        TouchI2c_Datastruct->TouchI2c_FM_State = TOUCHI2C_STATE_INIT;
        TouchI2c_Datastruct->touchI2C_Hw_State = TOUCHI2C_ERROR;
        return TouchI2c_Datastruct->touchI2C_Hw_State;
    }

    switch (TouchI2c_Datastruct->TouchI2c_FM_State)
    {
        case TOUCHI2C_STATE_INIT:
            I2c__Clear(TouchI2c_Datastruct->TouchI2c_Slave_Device);
            TouchI2c_Datastruct->TouchI2c_Hangup_Delay_Timeout = 0;
            response = I2c__Initialize(TouchI2c_Datastruct->TouchI2c_Slave_Device, TOUCH_I2C_SPEED, I2C_ADDR_7BITS,
            TouchI2c_Datastruct->slaveAddress);
            TouchI2c_Datastruct->touchI2C_Hw_State = TOUCHI2C_IDLE;
            TouchI2c_Datastruct->TouchI2c_FM_State = TOUCHI2C_STATE_READ_WRITE;
            break;

        case TOUCHI2C_STATE_READ_WRITE:
            if (TouchI2c_Datastruct->i2c_read_write == TOUCHI2C_READ_COMMAND)
            {
            	//! If a read command is requested, write the 16-bit offset address initially
                ret = I2c__Write(TouchI2c_Datastruct->TouchI2c_Slave_Device, TouchI2c_Datastruct->slaveOffsetMSB,
                &TouchI2c_Datastruct->slaveOffsetLSB, 1);

                TouchI2c_Datastruct->TouchI2c_FM_State = TOUCHI2C_STATE_REQUEST_READ;
            }
            //! Write command is requested
            else
            {
                ret = I2c__Write(TouchI2c_Datastruct->TouchI2c_Slave_Device, TouchI2c_Datastruct->slaveOffsetMSB,
                TouchI2c_Datastruct->src_dst_buffer_ptr, TouchI2c_Datastruct->number_Of_read_write_bytes);
                TouchI2c_Datastruct->TouchI2c_FM_State = TOUCHI2C_STATE_WAIT_WRITE_COMPLETION;
            }
            TouchI2c_Datastruct->touchI2C_Hw_State = TOUCHI2C_IN_PROGRESS;
            break;

        case TOUCHI2C_STATE_REQUEST_READ:
            ret = I2c__RequestRead(TouchI2c_Datastruct->TouchI2c_Slave_Device,
            TouchI2c_Datastruct->number_Of_read_write_bytes);
            TouchI2c_Datastruct->TouchI2c_FM_State = TOUCHI2C_STATE_REQUEST_DATA;
            TouchI2c_Datastruct->touchI2C_Hw_State = TOUCHI2C_IN_PROGRESS;
            break;

        case TOUCHI2C_STATE_WAIT_WRITE_COMPLETION:
            TouchI2c_Datastruct->TouchI2c_FM_State = TOUCHI2C_STATE_INIT;
            TouchI2c_Datastruct->touchI2C_Hw_State = TOUCHI2C_SUCCESS;
            break;

        case TOUCHI2C_STATE_REQUEST_DATA:
            ret = I2c__Read(TouchI2c_Datastruct->TouchI2c_Slave_Device, TouchI2c_Datastruct->src_dst_buffer_ptr,
            TouchI2c_Datastruct->number_Of_read_write_bytes);
            TouchI2c_Datastruct->TouchI2c_FM_State = TOUCHI2C_STATE_INIT;
            TouchI2c_Datastruct->touchI2C_Hw_State = TOUCHI2C_SUCCESS;
            break;

        default:
            TouchI2c_Datastruct->TouchI2c_FM_State = TOUCHI2C_STATE_INIT;

            TouchI2c_Datastruct->touchI2C_Hw_State = TOUCHI2C_IDLE;
            break;
    }
    return TouchI2c_Datastruct->touchI2C_Hw_State;
}

/**
 *    @brief    Checks if the heart beat from the slave device is incrementing during each check.
 *              If heart beat validation fails, I2C bus is reset and a callback function will be called
 *
 *    @param    none
 *
 *    @return   none
 *
 */
static void TouchI2c_HearBeatValidation(void)
{
   unsigned char devices;

    //! Wait for heart beat validation until timeout occurs
    if (TouchI2c_Heartbeat_delay_timeout)
    {
       TouchI2c_Heartbeat_delay_timeout--;
    }
    //! Validate heart beat for each slave device
    else
    {
        TouchI2c_Heartbeat_delay_timeout = TOUCHI2C_HEARBEAT_CHECK_TIMEOUT;

        for (devices = 0; devices < Cypress_Hardware_info.numberOfSlaves; devices++)
        {
        	//! Heart beat is incrementing, therefore store current heartbeat
            if (TouchI2c_HeartBeat_Slave_Counter[devices] != TouchI2c_Device_Status[devices].heartBeatCounterValue)
            {
                TouchI2c_HeartBeat_Slave_Counter[devices] = TouchI2c_Device_Status[devices].heartBeatCounterValue;
                TouchI2c_HeartBeat_Fail_Debounce_Counter[devices] = 0 ;
                TouchI2c_HeartBeat_Reset_Counter[devices]=0;
            }
            //! Heart beat is not incrementing, reset I2C bus
            else
            {
                TouchI2c_HeartBeat_Fail_Debounce_Counter[devices]++;

                //! Heart beat validation failed for more than debounce times, hence reset I2C bus to recover from any errors
                if (TouchI2c_HeartBeat_Fail_Debounce_Counter[devices] > TOUCHI2C_HEARTBEAT_FAIL_DEBOUNCE_COUNTER)
                {
                    TouchI2c_HeartBeat_Fail_Debounce_Counter[devices] = 0;
                    TouchI2c_HeartBeat_Slave_Counter[devices] = TouchI2c_Device_Status[devices].heartBeatCounterValue;
                    TouchI2c_FailureCallback(TOUCHI2C_HEARTBEAT_FAIL, devices, TouchI2c_Device_Status[devices].heartBeatCounterValue);

                    if (TouchI2c_HeartBeat_Reset_Counter[devices] < TOUCHI2C_KEYPAD_DISCONNECTED_DEBOUNCE)
                    {
                       TouchI2c_HeartBeat_Reset_Counter[devices]++;
                    }
                    else if(TouchI2c_HeartBeat_Reset_Counter[devices] == TOUCHI2C_KEYPAD_DISCONNECTED_DEBOUNCE)
                    {
                        API018Diagnostic__GenerateFault(FAULT_KEY_KEYPAD_DISCONNECTED);
                        TouchI2c_HeartBeat_Reset_Counter[devices]++;
                    }
                }
            }
        }
    }
}

/**
 *    @brief    Resets I2C slave device and clear I2C bus to recover from any error. Callback function
 *              will be called if declared
 *
 *    @param    error_type - Type of error occurred in the system
 *              slave_id - slave index of the device for which heart beat failed
 *              hearbeat_last_counter - Last valid heart beat value
 *
 *    @return   none
 *
 */
static void TouchI2c_FailureCallback(unsigned char error_type, unsigned char slave_id, unsigned int hearbeat_last_counter)
{
	//reset the bus and then call back if someone registered
    TouchI2c__ResetDevice(Cypress_Hardware_info.slaveAddressToBusMap[slave_id]);
	I2c__Clear(Cypress_Hardware_info.slaveAddressToBusMap[slave_id]);
	I2c__Initialize(Cypress_Hardware_info.slaveAddressToBusMap[slave_id], TOUCH_I2C_SPEED,I2C_ADDR_7BITS, Cypress_Hardware_info.slaveAddress[slave_id]);
	TOUCHI2C_ERROR_CALLBACK(error_type, slave_id, hearbeat_last_counter);
}

/**
 *    @brief    Returns the pointer to the Cypress slave hardware information data
 *
 *    @param    none
 *
 *    @return   Address of the structure which has Cypress slave hardware information
 *
 */
DEDICATED_TOUCH_INFO *TouchI2C__GetCYHardwareInfo(void)
{
	return((DEDICATED_TOUCH_INFO *)&Cypress_Hardware_info.numberOfSlaves);
}

/**
 *    @brief    Returns the pointer to the structure which has status report information
 *
 *    @param    slaveIndex - Index of the slave for which data is required
 *
 *    @return   Address of the structure which has status report information
 *
 */
TOUCHI2C_STATUS_REGS *TouchI2C__GetStatusReport(unsigned char slaveIndex)
{
	return((TOUCHI2C_STATUS_REGS *) &TouchI2c_Device_Status[slaveIndex].ctrlCommand);
}

/**
 *    @brief    Function updates the I2C command buffer to read either specific data or complete status report
 *
 *    @param    Read_SensorStatus - Index of the slave for which data is required
 *
 *    @return   Address of the structure which has status report information
 *
 */
void TouchI2C_Update_I2C_Cmd_Buffer_SensorStatus(T_BOOL Read_SensorStatus)
{
    unsigned char I2cDevice;

    //! Loop through all the devices and set the size and destination memory location
    for (I2cDevice = 0; I2cDevice < Cypress_Hardware_info.numberOfSlaves; I2cDevice++)
    {
		if (Read_SensorStatus == TRUE)
		{
			//! Configure the starting address
			TouchI2c_StateDeviceInfo[I2cDevice][0].slaveOffsetMSB = TOUCHI2C_STATUS_REPORT_SLAVE_OFFSET_MSB;
			TouchI2c_StateDeviceInfo[I2cDevice][0].slaveOffsetLSB = TOUCHI2C_STATUS_REPORT_SLAVE_OFFSET_LSB;

			//! Read complete sensor status report
			TouchI2c_StateDeviceInfo[I2cDevice][0].number_Of_read_write_bytes =	sizeof(TouchI2c_Device_Status[I2cDevice]);

			//! Set the destination address to starting of sensor report area
			TouchI2c_StateDeviceInfo[I2cDevice][0].src_dst_buffer_ptr =	(unsigned char*)&TouchI2c_Device_Status[I2cDevice].ctrlCommand;
		}
		else
		{
			//! Configure the starting address
			TouchI2c_StateDeviceInfo[I2cDevice][0].slaveOffsetMSB = TOUCHI2C_KEY_STATUS_SLAVE_OFFSET_MSB;
			TouchI2c_StateDeviceInfo[I2cDevice][0].slaveOffsetLSB = TOUCHI2C_KEY_STATUS_SLAVE_OFFSET_LSB;

			//! Read only key status from sensor status report
			TouchI2c_StateDeviceInfo[I2cDevice][0].number_Of_read_write_bytes = 6;

			// Set the destination address to starting of key status area
			TouchI2c_StateDeviceInfo[I2cDevice][0].src_dst_buffer_ptr = (unsigned char*) &TouchI2c_Device_Status[I2cDevice].touchKeyStatus[0];
		}
	}
}

/**
 *    @brief    Function returns the current heart beat value of touch i2c. heart beat value is used
 *              to confirm if the application is initiating i2c communication
 *
 *    @param    none
 *
 *    @return   Touch I2C heart beat counter
 *
 */
unsigned int TouchI2C__GetTouchI2CHeartbeatCounter(void)
{
	return(TouchI2C_Heartbeat_Counter);
}

unsigned char TouchI2C__GetSlaveIndexForChannel(unsigned char channelNum)
{
	unsigned char index;

	//! Loop through all the indexes in keysPerSlave array and find which index the key belongs to
	for (index = 0; index < Cypress_Hardware_info.numberOfSlaves; index++)
	{
		if (channelNum < TOUCHI2C_SlaveIndex_To_KeyNumRange[index])
			return(index);
	}

	//! Code should never come here
	return(-1);
}

/**
 *
 * @param i2c_busid
 * @return  Device Index of i2C Bus
 */
 unsigned char TouchI2C__GetI2cDeviceIndex(unsigned char i2c_busid)
{
    unsigned char device_index;

    DEDICATED_TOUCH_INFO *ptr2CypressHardwareInfo;
    ptr2CypressHardwareInfo =  TouchI2C__GetCYHardwareInfo();

    for(device_index=0; device_index < Cypress_Hardware_info.numberOfSlaves; device_index++ )
    {
        if(ptr2CypressHardwareInfo->slaveAddressToBusMap[device_index] == i2c_busid)
        {
            return device_index;
        }
    }
    return -1;
}





 /**
  *@ Brief :- This function is used to recover  Long key press or stuck. Using Baseline reset command  clear keystatus
  */
 static void TouchI2C_KeystuckRecoverHandler(void)
{
    // TouchI2c_KeyStuck_Fail_Counter[slaveindex];


    unsigned char ChannelNum;
    unsigned char ChannelIndex;
    unsigned char ActualChannelNum;
    unsigned char SlaveIndex;
    unsigned char keystatus_crc;
    unsigned char index;
    key_stuck_counter_index = 0;
    unsigned char *tempKeyStatus;


//! if  command TOUCHI2C_CMD_RESET_SENSOR_BSLN successfully write then clear SenderBase_Linecommand flag
    if (SenderBase_Linecommand)
    {
        if (TouchI2c_Device_Status[Baseline_Slaveindex].ctrlCommandEcho == TOUCHI2C_CMD_RESET_SENSOR_BSLN)
        {
        //! reset flag after succefully write
            SenderBase_Linecommand = 0;
		         //! Include this function macro TOUCHI2C__SENDKEYSTUCKINFORMATION() in  Touchi2c Prm File  
				//!Tis is used for send report to other node about key stuck report
				TOUCHI2C__SENDKEYSTUCKINFORMATION();
        }
        else
        {
        //! Retry counter added  if command not successfully
            BaselineReset_Timeout++;

            if (BaselineReset_Timeout >= BASELINE_RESET_TIMEOUT)
            {
                BaselineReset_Timeout = 0;
                SenderBase_Linecommand = 0;
				

            }
            else
            {
                return;
            }
        }
    }
    for (SlaveIndex = 0; SlaveIndex < Cypress_Hardware_info.numberOfSlaves; SlaveIndex++)
    {
//! verify Keystatus CRC 
        keystatus_crc = Crc8(0x00, &TouchI2c_Device_Status[SlaveIndex].touchKeyStatus[0],
        sizeof(TouchI2c_Device_Status[SlaveIndex].touchKeyStatus));

        if (keystatus_crc == TouchI2c_Device_Status[SlaveIndex].touchKeyCRC)
        {
            tempKeyStatus = &TouchI2c_Device_Status[SlaveIndex].touchKeyStatus[0];

//! calculate  channel index with respect to number of keys per slave devices
            for (ChannelNum = 0; ChannelNum < Cypress_Hardware_info.keysPerSlave[SlaveIndex]; ChannelNum++)
            {
                if (SlaveIndex > 0)
                {
                    ChannelIndex = Cypress_Hardware_info.keysPerSlave[0] + ChannelNum;
                }
                else
                {
                    ChannelIndex = ChannelNum;
                }
//! find out actual channel  from  channel index i.e. hardware map value
                ActualChannelNum = Cypress_Hardware_info.keysToSlaveKeyMapping[ChannelIndex];
 
 //! verify Keystatus  if any key press
                if (((tempKeyStatus[ActualChannelNum >> 3]) & (1 << (ActualChannelNum % 8))))
                {
                    if (key_stuck_counter_index < Cypress_Hardware_info.numberOfKeys)
                    {
 //!Check Long key  press/ stuck  counter for max on duration with respect to channel
                        TouchI2c_KeyStuck_Fail_Counter[key_stuck_counter_index]++;

                        if (TouchI2c_KeyStuck_Fail_Counter[key_stuck_counter_index] > (TouchI2c_FailurerecoverInterval[SlaveIndex] * TOUCHI2C_ONE_SECOND_TO_FIVE_MS_RATIO))
                        {
                            //
                            //CMD_RESET_SENSOR_BSLN: 0x14 - Command to reset specific sensor baseline
//! Send baseline reset command  to slave devices
                            if (TouchI2C__SendBaselineCommand(TOUCHI2C_CMD_RESET_SENSOR_BSLN, SlaveIndex, ActualChannelNum) == TRUE)
                            {
                                SenderBase_Linecommand = 1;
                                Baseline_Slaveindex = SlaveIndex;
                                BaselineReset_Timeout = 0;
                            }
                            else
                            {
                                TouchI2c_KeyStuck_Fail_Counter[key_stuck_counter_index]--;
                            }
                            return;
                        }
                        key_stuck_counter_index++;
                    }

                }
                else
                {
                    if (key_stuck_counter_index < Cypress_Hardware_info.numberOfKeys)
                    {
                        TouchI2c_KeyStuck_Fail_Counter[key_stuck_counter_index] = 0;

                        key_stuck_counter_index++;
                    }
                  //  TouchI2C_KeyStatus[ChannelIndex >> 3] &= ~(1 << (ChannelIndex % 8));
                }
            }
        }
        else
        {   //If CRC is not matching then reset keystatus buffer
            for (index = 0; index < sizeof(TouchI2c_Device_Status[SlaveIndex].touchKeyStatus); index++)
            {
                TouchI2c_Device_Status[SlaveIndex].touchKeyStatus[index] = 0x00;
                TouchI2C_KeyStatus[index] = 0x00;
            }
            key_stuck_counter_index = 0;
        }
    }


}
 //==========================================LPM MODE  function========================================================================================

#if(LOW_POWER_MODE_FEATURE==ENABLED)

 /*
  *
  * @Brief :- This function is used for Low power mode  purpose
  *  Enter into Low power mode
  */
//! This function is used  to enter low power mode
static  BOOL_TYPE TouchI2c_DeviceEnterLowPowerMode(void)
{
     BOOL_TYPE ret = FALSE, ret_i2c = FALSE;


     switch(Low_power_Mode_State)
     {
         default:
         case STATE_ENTER_LOW_POWER_IDLE:
             LowPower_Current_Device_Index = 0; //First device to be sent to low power
             Low_power_Mode_State = STATE_ENTER_LOW_POWER_SEND_CMD;
             break;

         case STATE_ENTER_LOW_POWER_STATUS_CHECK:
             if(GetPsoCDeviceMode(LowPower_Current_Device_Index) != CYPRESSTOUCH_MODE_INPROGRESS)
             {
                 if(Cypress_TouchPowerMode_Status[LowPower_Current_Device_Index] == CYPRESSTOUCH_MODE_ACTIVE_STATE)
                 {
                     Low_power_Mode_State = STATE_ENTER_LOW_POWER_SEND_CMD;
                 }
                 else
                 {
                     //Device is under low power, check next device
                     Low_power_Mode_State = STATE_ENTER_LOW_POWER_NEXT_DEVICE;
                 }
             }
             break;
         case STATE_ENTER_LOW_POWER_SEND_CMD:
             //! Send I2c command
             ret_i2c = TouchI2C__SendI2CCommand(TOUCHI2C_CMD_LOW_POWER_MODE, LowPower_Current_Device_Index);
             if (ret_i2c == TRUE)
             {
                 Device_LP_Status_Buffer = DEVICE_MODE_REQUEST;
                 Low_power_Mode_State = STATE_ENTER_LOW_POWER_STATUS_CHECK;
             }
             break;

         case STATE_ENTER_LOW_POWER_NEXT_DEVICE:
             LowPower_Current_Device_Index++;
             if(LowPower_Current_Device_Index >= Cypress_Hardware_info.numberOfSlaves)
             {
                 //All devices sent to low power
                 Low_power_Mode_State = STATE_ENTER_LOW_POWER_DONE;
             }
             else
             {
                 Device_LP_Status_Buffer = DEVICE_MODE_REQUEST;
                 Low_power_Mode_State = STATE_ENTER_LOW_POWER_STATUS_CHECK;
             }
             break;

         case STATE_ENTER_LOW_POWER_DONE:
             ret = TRUE;
             break;
     }

     return ret;
}



  /**
   *  This function is used for Low power mode debugging purpose
   * @Brief This function is used for Low power mode to active mode
   */
  static BOOL_TYPE TouchI2c_DeviceEnterActiveMode(void)
  {
   BOOL_TYPE ret = FALSE, ret_i2c = FALSE;

    switch(Active_Mode_State)
    {
        default:
        case STATE_ENTER_ACTIVE_IDLE:
            LowPower_Current_Device_Index = 0; //First device to be sent to low power
            Active_Mode_State = STATE_ENTER_ACTIVE_STATUS_CHECK;
            Device_LP_Status_Buffer = DEVICE_MODE_REQUEST;
            Send_Active_Cmd_Retry = 0;
            break;

        case STATE_ENTER_ACTIVE_STATUS_CHECK:
            if(GetPsoCDeviceMode(LowPower_Current_Device_Index) != CYPRESSTOUCH_MODE_INPROGRESS)
            {
                if(Cypress_TouchPowerMode_Status[LowPower_Current_Device_Index] == CYPRESSTOUCH_MODE_LOW_POWER_STATE)
                {
                    Active_Mode_State = STATE_ENTER_ACTIVE_SEND_CMD;
                    Send_Active_Cmd_Retry++;
                    if(Send_Active_Cmd_Retry > SEND_ACTIVE_RETRIES)
                    {
                        Send_Active_Cmd_Retry = 0;
                        Reset_I2c_Devices = RESET_I2C_TIME_MS;
                        Bkup_Lpm_time_Value = Lpm_Tick_time;
                        Active_Mode_State = STATE_ENTER_ACTIVE_BUS_RESET;
                    }
                }
                else
                {
                    //Device is under low power, check next device
                    Active_Mode_State = STATE_ENTER_ACTIVE_NEXT_DEVICE;
                }
            }
            break;

        case STATE_ENTER_ACTIVE_SEND_CMD:
            //! Send I2c command
            ret_i2c = TouchI2C__SendI2CCommand(TOUCHI2C_CMD_ACTIVE_MODE, LowPower_Current_Device_Index);
            if (ret_i2c == TRUE)
            {
                Device_LP_Status_Buffer = DEVICE_MODE_REQUEST;
                Active_Mode_State = STATE_ENTER_ACTIVE_STATUS_CHECK;
            }
            break;

        case STATE_ENTER_ACTIVE_NEXT_DEVICE:
            LowPower_Current_Device_Index++;
            if(LowPower_Current_Device_Index >= Cypress_Hardware_info.numberOfSlaves)
            {
                //All devices sent to low power
                Active_Mode_State = STATE_ENTER_ACTIVE_DONE;
            }
            else
            {
                Device_LP_Status_Buffer = DEVICE_MODE_REQUEST;
                Active_Mode_State = STATE_ENTER_ACTIVE_STATUS_CHECK;
            }
            break;

        case STATE_ENTER_ACTIVE_DONE:
            ret = TRUE;
            break;

        case STATE_ENTER_ACTIVE_BUS_RESET:
            if(Reset_I2c_Devices == RESET_I2C_NONE)
            {
                Active_Mode_State = STATE_ENTER_ACTIVE_SEND_CMD;
            }
            break;
    }

    return ret;
}


  /**
    *   Check the Cypress Device  Mode status
    *    Read request status
    */
   static CYPRESS_TOUCH_POWER_MODE_STATUS GetPsoCDeviceMode(uint8 deviceIndex)
  {

       PASS_FAIL_TYPE ret = FAIL;

       switch (Device_LP_Status_Buffer)
       {
           default:
           case DEVICE_MODE_REQUEST:
               ret = TouchI2C__GetWidgetEnableBufferRequestInfo(deviceIndex);
               Cypress_TouchPowerMode_Status[deviceIndex] = CYPRESSTOUCH_MODE_INPROGRESS;
               if (ret == PASS)
               {
                   Bkup_Lpm_time_Value = Lpm_Tick_time;
                   Device_LP_Status_Buffer = DEVICE_COMPLETE_RESPONSE;
               }
               break;

           case DEVICE_COMPLETE_RESPONSE:
               Cypress_TouchPowerMode_Status[deviceIndex] = CYPRESSTOUCH_MODE_INPROGRESS;
               if (TouchI2c__GetDeviceReadWriteCmdRequestStatus((deviceIndex)) == 0)
               {
                   memcpy(&WidgetEnable_LPM_Buffer,&Widget_Enable_Buffer_Info[deviceIndex].Widget_enable_Lp_Mode_Buffer[0],sizeof(WidgetEnable_LPM_Buffer));

                   /* If any of the non low-power mode keys are enabled */
                #if (GESE_SETTING_TOUCH_PARAMETERS != ENABLED)
                   (WidgetEnable_LPM_Buffer & (~LOW_POWER_ENABLED_BUTTONS_MASK_SLAVE_0))?(Cypress_TouchPowerMode_Status[deviceIndex]  = CYPRESSTOUCH_MODE_ACTIVE_STATE):(Cypress_TouchPowerMode_Status[deviceIndex] = CYPRESSTOUCH_MODE_LOW_POWER_STATE);
                   Device_LP_Status_Buffer = DEVICE_IDLE;
                #else
                   //The device configured to monitor keys during low power has a specific condition based on the Low_Power_SensorNumber
                   if(deviceIndex == Low_power_Device_index)
                   {
                       if((WidgetEnable_LPM_Buffer == Low_Power_SensorNumber) || (WidgetEnable_LPM_Buffer == 0))
                       {
                           Cypress_TouchPowerMode_Status[deviceIndex] = CYPRESSTOUCH_MODE_LOW_POWER_STATE;
                       }
                       else
                       {
                           Cypress_TouchPowerMode_Status[deviceIndex] = CYPRESSTOUCH_MODE_ACTIVE_STATE;
                       }
                   }
                   else //other devices simply should be compared to ZERO (none sensors)
                   {
                       if(WidgetEnable_LPM_Buffer!=0)
                       {
                           Cypress_TouchPowerMode_Status[deviceIndex] = CYPRESSTOUCH_MODE_ACTIVE_STATE;
                       }
                       else
                       {
                           Cypress_TouchPowerMode_Status[deviceIndex] = CYPRESSTOUCH_MODE_LOW_POWER_STATE;
                       }
                   }
                   Device_LP_Status_Buffer = DEVICE_IDLE;
                #endif
               }
               else if ((SystemTimer_GetTimeElapsed(Lpm_Tick_time, Bkup_Lpm_time_Value) > 20))
               {
                   Device_LP_Status_Buffer = DEVICE_MODE_REQUEST;
               }
               break;

           case DEVICE_IDLE:
               break;
       }

       return Cypress_TouchPowerMode_Status[deviceIndex];
   }


   /**
    * @brief   This function is used to  request  Cypress into Low power mode or Active Mode using request set function
    *  With respect to request state Cypress devcie goes to Low power or Active Mode
    *  Set values  :-  CYPRESS_REQUEST_TO_ACTIVE_MODE=0 ,CYPRESS_REQUEST_TO_LOW_POWER_MODE=1
    * @return  BOOL_TYPE: returns result
    */
   BOOL_TYPE TouchI2c__SetLowPowerRequestState(CYPRESS_TOUCH_POWERMODE_REQUEST_STATE set_request_low_power_mode)
   {
       BOOL_TYPE result = FALSE;
       if((Cypress_TouchPowerMode_Request_State == CYPRESS_REQUEST_MODE_COMPLETE)&&(set_request_low_power_mode < CYPRESS_REQUEST_MODE_COMPLETE))
       {
           //!  Currently not trying to enter any mode and we get a valid mode request
           Cypress_TouchPowerMode_Request_State = set_request_low_power_mode;
           result = TRUE;
       }

       return result;
   }


   /**
    * @brief   This function is used to  Get Status of Cypress state i.e Low power mode or Active Mode
    *
    *  Read Status Value  i.e.
    *   CYPRESSTOUCH_CURRENTSTATE_LOW_POWER_MODE_STATE = 1,
    *  CYPRESSTOUCH_CURRENTSTATE_ACTIVE_MODE_STATE = 2,
    * @return  BOOL_TYPE: returns  Power mode status
    */
   CYPRESS_TOUCH_CURRENT_STATE TouchI2c__GetTouchPowerStatus(void)
   {

       return Cypress_Touch_Current_State;
   }



   /**
    * @Brief :- request Widget Enable LPM Mode buffer w .r. t slave
    *Read Widget enable LPM MODE buffer data  from  Cypress Device
    */

   PASS_FAIL_TYPE TouchI2C__GetWidgetEnableBufferRequestInfo(unsigned char deviceindex)
    {
          const DEDICATED_TOUCH_INFO *ptr2Cypress_Hw_info;
         PASS_FAIL_TYPE type;
          ptr2Cypress_Hw_info =  TouchI2C__GetCYHardwareInfo();

       if (TouchI2c__GetDeviceReadWriteCmdRequestStatus(deviceindex) == 0)
       {
         TOUCHI2C_COMMAND_STRUCT sensorstatusrequest;
          sensorstatusrequest.TouchI2c_Slave_Device = ptr2Cypress_Hw_info->slaveAddressToBusMap[deviceindex];
          sensorstatusrequest.slaveAddress = (((ptr2Cypress_Hw_info->slaveAddress[deviceindex]) ^ 0x02) << 1);
          sensorstatusrequest.slaveOffsetMSB =0x00;
          sensorstatusrequest.slaveOffsetLSB =0x0C;
          sensorstatusrequest.i2c_read_write= TOUCHI2C_READ_COMMAND;
          sensorstatusrequest.number_Of_read_write_bytes =4;
          sensorstatusrequest.src_dst_buffer_ptr = &Widget_Enable_Buffer_Info[deviceindex].Widget_enable_Lp_Mode_Buffer[0];
          sensorstatusrequest.touchI2C_Hw_State = TOUCHI2C_SUCCESS;
          sensorstatusrequest.Send_Command_Continously = 0;
          sensorstatusrequest.IscmdRequest = 1;
          sensorstatusrequest.TouchI2c_Hangup_Delay_Timeout = 100;
          sensorstatusrequest.TouchI2c_FM_State = TOUCHI2C_STATE_INIT;

        //   type= SRTouchI2c__ReadRequestForSlaveDevice(&sensorstatusrequest,deviceindex);

           return TouchI2c__ReadRequestForSlaveDevice(&sensorstatusrequest, deviceindex);
         }
		 return FAIL;

    }

    /**
     * @brief    Returns the values of Read Widget enable LPM MODE buffer data  from  Cypress Device
     * @param slaveIndex
     * @return
     */
    WIDGET_ENABLE_LPMODE_BUFFER_INFO *TouchI2C_GetWidgetEnableLPMModeBufferData(unsigned char slaveIndex)
    {
        if (slaveIndex < Cypress_Hardware_info.numberOfSlaves)
        {
            return ((WIDGET_ENABLE_LPMODE_BUFFER_INFO *) &Widget_Enable_Buffer_Info[slaveIndex].Widget_enable_Lp_Mode_Buffer);
        }
        return 0;
    }


#endif


// Generic function created getTime elapsed
static uint8 SystemTimer_GetTimeElapsed(uint8 currentTimeStamp, uint8 prevTimeStamp)

{
    uint8 timeElapsed;

    if (currentTimeStamp >= prevTimeStamp)
    {
      timeElapsed = currentTimeStamp - prevTimeStamp;
    }
    else
    {
      timeElapsed = (255 - prevTimeStamp) + currentTimeStamp;
    }
    return timeElapsed;

}



/**
 * @brief   This function update key status  with respect to slave devices
 *
 * @return NA
 */

static void  TouchI2cUpdateStatus(void)
{
    uint8 *tempKeyStatus;
    uint8 ChannelNum;
    uint8 ChannelIndex;
    uint8 ActualChannelNum;
    uint8 SlaveIndex;
    uint8  keystatus_crc;
    uint8 index;


    for(SlaveIndex = 0; SlaveIndex < Cypress_Hardware_info.numberOfSlaves; SlaveIndex++)
    {
        keystatus_crc = Crc8(0x00, &TouchI2c_Device_Status[SlaveIndex].touchKeyStatus[0], sizeof(TouchI2c_Device_Status[SlaveIndex].touchKeyStatus));

        if(keystatus_crc ==TouchI2c_Device_Status[SlaveIndex].touchKeyCRC)
        {

            tempKeyStatus = &TouchI2c_Device_Status[SlaveIndex].touchKeyStatus[0];

            for (ChannelNum = 0; ChannelNum < Cypress_Hardware_info.keysPerSlave[SlaveIndex]; ChannelNum++)
            {
                if (SlaveIndex > 0)
                {
                    ChannelIndex = Cypress_Hardware_info.keysPerSlave[0] + ChannelNum;
                }
                else
                {
                    ChannelIndex = ChannelNum;
                }

                ActualChannelNum = Cypress_Hardware_info.keysToSlaveKeyMapping[ChannelIndex];

                if ((tempKeyStatus[ActualChannelNum >> 3]) & (1 << (ActualChannelNum % 8)))
                {
                    TouchI2C_KeyStatus[ChannelIndex >> 3] |= (1 << (ChannelIndex % 8));
                }
                else
                {
                    TouchI2C_KeyStatus[ChannelIndex >> 3] &= ~(1 << (ChannelIndex % 8));
                }
            }
        }
        else
        {   //If CRC is not matching then reset keystatus buffer
            for (index = 0; index < sizeof(TouchI2c_Device_Status[SlaveIndex].touchKeyStatus); index++)
            {
                TouchI2c_Device_Status[SlaveIndex].touchKeyStatus[index] = 0x00;
                TouchI2C_KeyStatus[index]=0x00;
            }
        }
    }

}

#endif  //#if(TOUCH_I2C_PROTOCOL == ENABLED)
