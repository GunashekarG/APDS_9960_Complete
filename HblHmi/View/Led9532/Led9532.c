/**
 *  @file
 *
 *  @brief      Handler for OnSemi CAT9532 LED Driver over I2c
 *
 *  @details    This module can control 1 or more 9532 LED Drivers from OnSemi
 *
 *  $Header: Led9532.c 1.7 2015/06/27 19:49:27EDT BRUNO LUIZ BITTENCOURT (BITTEBL) Exp  $
 *
 *  @copyright  Copyright 2013-$Date: 2015/06/27 19:49:27EDT $. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
//-------------------------------------- Include Files ----------------------------------------------------------------
#include "SystemConfig.h"
#include "Led9532.h"

#if (LED_DRIVER_CAT9532 == ENABLED)
#include "Led9532_prv.h"

#include "I2c.h"
#include "I2cMgr.h"
#include <string.h>

//-------------------------------------- PRIVATE (Variables, Constants & Defines) -------------------------------------
#if(LED9532_READ_FEATURE == DISABLED)
#define LED9532_PERIODIC_READ_FEATURE           DISABLED
#define LED9532_NOISE_IMMUNITY_FEATURE          DISABLED
#define LED9532_READ_AFTER_EACH_WRITE_FEATURE   DISABLED
#else
#if(LED9532_NOISE_IMMUNITY_FEATURE  == ENABLED)
   #define LED9532_PERIODIC_READ_FEATURE        ENABLED
#endif  //LED9532_NOISE_IMMUNITY_FEATURE
#endif  //LED9532_READ_FEATURE

static const LED9532_DEVICE_TYPE LED9532_DEVICES[LED9532_NUM_DEVICES] = LED9532_DEVICE_LIST;
static const LED9532_IO_TYPE LED9532_IO_CONFIG[LED9532_NUM_DEVICES] = LED9532_IO_CONFIG_LIST;

static LED9532_STATE_TYPE Led9532_State;
static uint8 Led9532_Device_Index;

static BOOL_TYPE New_Device_Data;
static uint8 Led9532_TimeOut;

static LED9532_MEM_TYPE Led9532_Buffer[LED9532_NUM_DEVICES];
static LED9532_MEM_TYPE Led9532_BackupBuffer[LED9532_NUM_DEVICES];

#if(LED9532_READ_FEATURE == ENABLED)
BOOL_TYPE Led9532_Read_Option[LED9532_NUM_DEVICES];
static uint8 Led9532_Device_Read_Index;
static LED9532_READ_MEM_TYPE Led9532_ReadBuffer[LED9532_NUM_DEVICES];
static LED9532_READ_STATUS_TYPE Led9532_Read_Status[LED9532_NUM_DEVICES];
#if(LED9532_PERIODIC_READ_FEATURE == ENABLED)
static const uint16 Led9532_Read_Interval[LED9532_NUM_DEVICES] = LED9532_READ_INTERVAL_LIST;
static uint16 Led9532_Read_Interval_Counter[LED9532_NUM_DEVICES];
#endif  //LED9532_PERIODIC_READ_FEATURE
#if(LED9532_NOISE_IMMUNITY_FEATURE  == ENABLED)
static uint8 Led9532_Read_Error_Counter;
#endif  //LED9532_NOISE_IMMUNITY_FEATURE
#endif  //LED9532_READ_FEATURE
static uint8 Led9532_DeviceHandle[LED9532_NUM_DEVICES];

//-------------------------------------- PRIVATE (Function Prototypes) ------------------------------------------------
//=====================================================================================================================
//-------------------------------------- Public Functions -------------------------------------------------------------
//=====================================================================================================================

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      It Initializes the module Led95x and its variables
 *
 */
void Led9532__Initialize(void )
{
    uint8 index;
    Led9532_State = LED9532_STATE_IDLE;
    Led9532_Device_Index = 0;
    New_Device_Data = FALSE;
    Led9532_TimeOut = 0;
    memset(Led9532_Buffer, 0, sizeof(Led9532_Buffer));
    memset(Led9532_BackupBuffer, 0, sizeof(Led9532_BackupBuffer));
    memset(Led9532_DeviceHandle, I2CMGR_HANDLE_INVALID, sizeof(Led9532_DeviceHandle));

#if(LED9532_READ_FEATURE == ENABLED)
    Led9532_Device_Read_Index = 0;
    memset(Led9532_ReadBuffer, 0, sizeof(Led9532_ReadBuffer));
#endif  //LED9532_READ_FEATURE
    for (index = 0; index < LED9532_NUM_DEVICES; index++)
    {
        Led9532_DeviceHandle[index] =  I2cMgr__GetDeviceHandle(LED9532_DEVICES[index].i2c_channel);

        Led9532_BackupBuffer[index].psc0 = 1; // to force write 0 and set the frequency to 152Hz
        Led9532_BackupBuffer[index].psc1 = 1; // to force write 0 and set the frequency to 152Hz
#if(LED9532_READ_FEATURE == ENABLED)
        Led9532_Read_Option[index] = FALSE;
        Led9532_Read_Status[index] = LED9532_READ_NOT_STARTED;
#if(LED9532_PERIODIC_READ_FEATURE == ENABLED)
        Led9532_Read_Interval_Counter[index] = Led9532_Read_Interval[index];
#endif  //LED9532)PERIODIC_READ_FEATURE
#endif //LED9532_READ_FEATURE
    }
#if(LED9532_NOISE_IMMUNITY_FEATURE  == ENABLED)
        Led9532_Read_Error_Counter = LED9532_READ_ERROR_COUNTER_MAX;
#endif  //LED9532_NOISE_IMMUNITY_FEATURE

}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief This method handles the communication with the LED9532 driver over the I2c Bus
 * @details This Handler returns a value which can be used to safely multiple different devices on the I2c Bus
 * @return  LED9532_STATUS_TYPE
 *      @retval LED9532_STATUS_BUSY which means the process is in progress and can not be interrupted by another device in the I2c bus.
 *      @retval LED9532_STATUS_FREE which means the previews process is finished with no errors and the I2c bus can be handle to another device on the bus.
 *      @retval LED9532_STATUS_ERROR which means the previews process is finished with errors but the I2c bus can be handled to another device on the bus.
 */
LED9532_STATUS_TYPE Led9532__Handler(void )
{
    LED9532_STATUS_TYPE response;
    uint8 device_index;
    response = LED9532_STATUS_BUSY;

    switch (Led9532_State)
    {
         case LED9532_STATE_IDLE:
            // if there is data changed for the considered device.
            // Any change on the PWM or LED will be detected resulting in a write procedure to that device.
             if (New_Device_Data == TRUE)
             {
                 New_Device_Data = FALSE;   //This allows the LED9532_READ_FEATURE to be executed
                 for(device_index = 0; device_index < LED9532_NUM_DEVICES; device_index++)
                 {
                     if (memcmp(&Led9532_Buffer[device_index], &Led9532_BackupBuffer[device_index],
                     sizeof(LED9532_MEM_TYPE)) != 0)
                     {
                         if(I2cMgr__RequestBus(Led9532_DeviceHandle[device_index]) == TRUE)
                         {
                             // update the data buffer
                             memcpy(&Led9532_BackupBuffer[device_index], &Led9532_Buffer[device_index],
                             sizeof(LED9532_MEM_TYPE));
                             // set the I2c
                             I2c__Initialize(LED9532_DEVICES[device_index].i2c_channel, LED9532_I2C_SPEED, I2C_ADDR_7BITS,
                             LED9532_DEVICES[device_index].i2c_address);
                             Led9532_State = LED9532_STATE_WRITE;
                             Led9532_TimeOut = LED9532_TIMEROUT;
                             Led9532_Device_Index = device_index;
                             break;
                         }
                     }
                 }
             }
             else
             {
#if(LED9532_READ_FEATURE == ENABLED)
                 Led9532_Device_Read_Index++;
                 if (Led9532_Device_Read_Index >= LED9532_NUM_DEVICES) // scan over all devices
                 {
                     Led9532_Device_Read_Index = 0;
                 }
                 if(Led9532_Read_Option[Led9532_Device_Read_Index] == TRUE)
                 {
                     Led9532_Read_Option[Led9532_Device_Read_Index] = FALSE;
                     Led9532_State = LED9532_READ_INITIALIZE;
                     Led9532_TimeOut = LED9532_TIMEROUT;
                     Led9532_Device_Index = Led9532_Device_Read_Index;
                     break;
                 }
                 else
#endif  //LED9532_READ_FEATURE
                 {
                     response = LED9532_STATUS_FREE;
                 }
             }
            break;
        case LED9532_STATE_WRITE:
            if (I2c__GetStatus(LED9532_DEVICES[Led9532_Device_Index].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                // perform the write
                I2c__Write(LED9532_DEVICES[Led9532_Device_Index].i2c_channel, LED9532_WRITE_REGISTER_ADDRESS,
                (uint8 *) &Led9532_BackupBuffer[Led9532_Device_Index], sizeof(LED9532_MEM_TYPE));
                Led9532_State = LED9532_STATE_WAIT_WRITE_END;
#if(LED9532_READ_AFTER_EACH_WRITE_FEATURE == ENABLED)
                Led9532_Read_Option[Led9532_Device_Index] = TRUE;
#endif  //LED9532_READ_AFTER_EACH_WRITE_FEATURE
            }
            break;
        case LED9532_STATE_WAIT_WRITE_END:
            if (I2c__GetStatus(LED9532_DEVICES[Led9532_Device_Index].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                I2cMgr__ReleaseBus(Led9532_DeviceHandle[Led9532_Device_Index]);
#if(LED9532_READ_FEATURE == ENABLED)
                if(Led9532_Read_Option[Led9532_Device_Index] == TRUE)
            	{
                    Led9532_Read_Option[Led9532_Device_Index] = FALSE;
                    Led9532_State = LED9532_READ_INITIALIZE;
                    Led9532_TimeOut = LED9532_TIMEROUT;
                }
                else
#endif  //LED9532_READ_FEATURE
                {
                    Led9532_State = LED9532_STATE_IDLE;
                    response = LED9532_STATUS_FREE;
                }
            }
            break;
#if(LED9532_READ_FEATURE == ENABLED)
        case LED9532_READ_INITIALIZE:
            if (I2c__GetStatus(LED9532_DEVICES[Led9532_Device_Index].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                if(I2cMgr__RequestBus(Led9532_DeviceHandle[device_index]) == TRUE)
                {
                    I2c__Initialize(LED9532_DEVICES[Led9532_Device_Index].i2c_channel, LED9532_I2C_SPEED, I2C_ADDR_7BITS,
                    LED9532_DEVICES[Led9532_Device_Index].i2c_address);
                    I2c__Write(LED9532_DEVICES[Led9532_Device_Index].i2c_channel, LED9532_READ_REGISTER_ADDRESS , NULL,0);
                    Led9532_State = LED9532_STATE_READ;
                    Led9532_Read_Status[Led9532_Device_Index] = LED9532_READ_IN_PROGRESS; //Read process started
#if(LED9532_NOISE_IMMUNITY_FEATURE == ENABLED)
                    Led9532_Read_Error_Counter = LED9532_READ_ERROR_COUNTER_MAX;  //Load the error counter value
                }
#endif  //LED9532_NOISE_IMMUNITY_FEATURE
            }
            break;
        case LED9532_STATE_READ:
            if ((I2c__GetStatus(LED9532_DEVICES[Led9532_Device_Index].i2c_channel,I2C_STATUS_ERROR)) == I2C_ERROR_ACK)
            {
            	response = LED9532_STATUS_ERROR;
            	Led9532_State = LED9532_STATE_IDLE;
            	Led9532_Read_Status[Led9532_Device_Index] = LED9532_READ_ERROR;
            }
            if (I2c__GetStatus(LED9532_DEVICES[Led9532_Device_Index].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                // perform the Read
                I2c__RequestRead(LED9532_DEVICES[Led9532_Device_Index].i2c_channel,sizeof(LED9532_READ_MEM_TYPE));
                Led9532_State = LED9532_STATE_WAIT_READ_END;
            }
            break;
        case LED9532_STATE_WAIT_READ_END:
            if (I2c__GetStatus(LED9532_DEVICES[Led9532_Device_Index].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
            	I2c__Read(LED9532_DEVICES[Led9532_Device_Index].i2c_channel,(uint8 *)&Led9532_ReadBuffer[Led9532_Device_Index],sizeof(LED9532_READ_MEM_TYPE));
            	I2cMgr__ReleaseBus(Led9532_DeviceHandle[Led9532_Device_Index]);
                Led9532_Read_Status[Led9532_Device_Index] = LED9532_READ_COMPLETED;
 #if (LED9532_NOISE_IMMUNITY_FEATURE == ENABLED)
                if (memcmp(&Led9532_BackupBuffer[Led9532_Device_Index], &Led9532_ReadBuffer[Led9532_Device_Index].rw_registers, sizeof(LED9532_MEM_TYPE)) != 0)
                {
                	//If there is change in the data read from device compared to the last written data, re-write the device again
                    if (Led9532_Read_Error_Counter) //check for max re-write try
                    {
                    	if(I2cMgr__RequestBus(Led9532_DeviceHandle[device_index]) == TRUE)
                    	{
                    	    Led9532_Read_Error_Counter--;
                            I2c__Initialize(LED9532_DEVICES[Led9532_Device_Index].i2c_channel, LED9532_I2C_SPEED, I2C_ADDR_7BITS,
                            LED9532_DEVICES[Led9532_Device_Index].i2c_address);
                            Led9532_State = LED9532_STATE_WRITE;
                            Led9532_TimeOut = LED9532_TIMEROUT;
                    	}
                    }
                    else
                    {
                        Led9532_State = LED9532_STATE_IDLE;
                        response = LED9532_STATUS_ERROR;
                    }
                }
                else    // Reading finished successfully
#endif  //LED9532_NOISE_IMMUNITY_FEATURE
                {
                Led9532_State = LED9532_STATE_IDLE; // when finished go back to idle state
                response = LED9532_STATUS_FREE;
                LED9532_READING_COMPLETE_CALLBACK(Led9532_Device_Index);// Callback function after completing read operation
                }
            }
            break;
#endif //LED9532_READ_FEATURE
        default:
            Led9532_State = LED9532_STATE_IDLE;
            response = LED9532_STATUS_FREE;
            break;
    }
#if(LED9532_PERIODIC_READ_FEATURE == ENABLED)
    if(Led9532_Read_Interval_Counter[Led9532_Device_Read_Index] != 0)
    {
    	Led9532_Read_Interval_Counter[Led9532_Device_Read_Index]--;
    }

    else if (Led9532_Read_Status[Led9532_Device_Read_Index] != LED9532_READ_IN_PROGRESS)
    {
    	Led9532_Read_Option[Led9532_Device_Read_Index] = TRUE;
       	Led9532_Read_Interval_Counter[Led9532_Device_Read_Index] = Led9532_Read_Interval[Led9532_Device_Read_Index]; //Reload the read interval counter
    }
#endif  //LED9532_PERIODIC_READ_FEATURE

    if(response == LED9532_STATUS_BUSY)
    {
        if (Led9532_TimeOut)
        {
            Led9532_TimeOut--;
        }
        else

        {
#if(LED9532_AUTOREWRITE_FEATURE == ENABLED)
        	if(Led9532_State == LED9532_STATE_WRITE || Led9532_State == LED9532_STATE_WAIT_WRITE_END)// Check if i2c error happened during write operation
        	{
                Led9532_BackupBuffer[Led9532_Device_Index].ls[0] = ~Led9532_Buffer[Led9532_Device_Index].ls[0];// Facilitating forced re-write of device in case of write error
        	}
#endif  //LED9532_AUTOREWRITE_FEATURE
            Led9532_State = LED9532_STATE_IDLE;
            response = LED9532_STATUS_ERROR;
        }
    }
    return(response);
}


//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief This method forces a write into a specific device.
 * @details Do not call this method if you are using the Led9532__Handler method.
 * @param device
*  @retval TRUE which means the the requested i2c write operation cannot be performed since i2c bus is busy
*  @retval FALSE which means the requested i2c write operation is completed
*/
BOOL_TYPE Led9532__ForceWrite2Device(uint8 device )
{
    // update the data buffer
    BOOL_TYPE response;

    response = TRUE;

    I2c__Initialize(LED9532_DEVICES[device].i2c_channel, LED9532_I2C_SPEED, I2C_ADDR_7BITS,LED9532_DEVICES[device].i2c_address);
    if (I2c__GetStatus(LED9532_DEVICES[Led9532_Device_Index].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
    {
        I2c__Write(LED9532_DEVICES[device].i2c_channel, 0x12, (uint8 *) &Led9532_Buffer[device],sizeof(LED9532_MEM_TYPE));
        response = FALSE;
    }
    return response;
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief This method is used to check the state of the BUS
 * @param device
 * @return state of the bus
 *  @retval FALSE -  bus free
 *  @retval TRUE  -  bus busy
 */
BOOL_TYPE Led9532__CheckForcedWrite(uint8 device )
{
    return ((BOOL_TYPE) I2c__GetStatus(LED9532_DEVICES[device].i2c_channel, I2C_STATUS_STATE));
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief This method is used to set the value for one of the 2 internal PWMs
 * @param device    define which device is begin considered
 * @param pwm       define which pwm of the selected device is being considered
 * @param intensity define the duty-cycle percentage
 */
void Led9532__SetIntensity(uint8 device, LED9532_PWM_TYPE pwm, uint8 intensity)
{
    if (device < LED9532_NUM_DEVICES)
    {
        if (pwm == LED9532_PWM0)
        {
            Led9532_Buffer[device].pwm0 = LED9532_PWM_COUNT_TABLE[intensity * sizeof(LED9532_PWM_COUNT_TABLE) / 100];        // Adjust pwm counts based on number of pwm selections
        }
        else
        {
            Led9532_Buffer[device].pwm1 = LED9532_PWM_COUNT_TABLE[intensity * sizeof(LED9532_PWM_COUNT_TABLE) / 100];        // Adjust pwm counts based on number of pwm selections
        }
        New_Device_Data = FALSE;
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief This method connected a specific output with one of the possible options.
 * @param device define which device is begin considered
 * @param led    define the specific output of the selected device.
 * @param state define where the output should be connected to (LED9532_LED_OFF, LED9532_LED_ON, LED9532_LED_PWM0 or  LED9532_LED_PWM1)
 *
 */
void Led9532__SetLed(uint8 device ,uint8 led ,LED9532_LED_TYPE state )
{
    uint8 aux;
    if ((device < LED9532_NUM_DEVICES) && (led < LED9532_NUM_LEDS) && (state <= LED9532_LED_PWM1))
    {
    	if((LED9532_IO_CONFIG[device].port_config & (1 << led)) == OUTPUT) // Allow LED write operation only if that IO is configured as output
    	{
            aux = ~(3 << ((led & 3) << 1));
            aux &= Led9532_Buffer[device].ls[led >> 2]; // clear the bits related to the led.
            aux |= (state << ((led & 3) << 1));
            Led9532_Buffer[device].ls[led >> 2] = aux; // set the state into the bits related to the led.
            New_Device_Data = TRUE;
    	}
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief  Function to copy passed buffer directly to LED9532 buffer.
 * @param  Device selected
 *
 */
void Led9532__SetLedBuffer(LED9532_MEM_TYPE *led_buffer ,uint8 device )
{
    if (device < LED9532_NUM_DEVICES)
    {
        memcpy(&Led9532_Buffer[device], led_buffer, sizeof(LED9532_MEM_TYPE));
    }
}

#if(LED9532_READ_FEATURE == ENABLED)
//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief  Function to read input registers of LED9532.
 * @param  Device selected
 * @return Input Register Value
 */
uint16 Led9532__ReadInputRegister(uint8 device)
{
	return ((uint16)(Led9532_ReadBuffer[device].input_reg1 << 8 | Led9532_ReadBuffer[device].input_reg0 ));
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief  Function Sets the read option for LED9532 device.
 * @param  Device selected
 *
 */
void Led9532__SendReadCommand(uint8 device)
{
	Led9532_Read_Option[device] = TRUE;
}
//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief  Function to get read status
 * @param  Device selected
 * @return Read Status
 */
LED9532_READ_STATUS_TYPE Led9532__GetReadStatus(uint8 device)
{
	LED9532_READ_STATUS_TYPE ret_value;
	ret_value = Led9532_Read_Status[device];
	if(ret_value == LED9532_READ_COMPLETED)
	{
		Led9532_Read_Status[device] = LED9532_READ_NOT_STARTED ;
	}
	return ret_value;
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief  Function to read a specific input pin of LED9532.
 * @param  Device selected
 * @return Input Pin Status
 *
 */
LED9532_INPUT_TYPE Led9532__ReadInputPin(uint8 device, uint8 pin)
{
	LED9532_INPUT_TYPE ret_value;
	uint16 input_register;
	input_register = Led9532__ReadInputRegister(device);
	if((LED9532_IO_CONFIG[device].port_config & (1 << pin)) == OUTPUT)
	{
		ret_value = LED9532_INPUT_INVALID; // return Invalid, if the pin is configured as output
	}
	else if(input_register & (1 << pin))
	{
		ret_value = LED9532_INPUT_HIGH;
	}
	else
	{
		ret_value = LED9532_INPUT_LOW;
	}
	return ret_value;
}
//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief  Function to get buffer which is read from LED9532.
 * @param  Device selected
 *
 */
void Led9532__GetReadBuffer(LED9532_READ_MEM_TYPE *read_buffer ,uint8 device )
{
    if (device < LED9532_NUM_DEVICES)
    {
        memcpy(read_buffer, &Led9532_ReadBuffer[device], sizeof(LED9532_READ_MEM_TYPE));
    }
}
#endif  //LED9532_READ_FEATURE
#endif  //LED_DRIVER_CAT9532

