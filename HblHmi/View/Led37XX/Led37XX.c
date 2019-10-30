/**
 *  @file       Led37XX.c
 *
 *  @brief      Handler for ISSI 31FL37XX LED Driver over I2c
 *
 *  @details    This module can control 1 or more 37XX LED Drivers from ISSI
 *
 *
 *  $Header:    Led37XX.c 1.0 2017/03/20 16:00:00EST Ben Wang (WANGB9) Exp  $
 *
 *  @copyright  Copyright 2013-$Date: 2017/03/20 16:00:00EST $. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------

//-------------------------------------- Include Files ----------------------------------------------------------------
#include "SystemConfig.h"
#include "Led37XX.h"

//-------------------------------------- PRIVATE (Variables, Constants & Defines) -------------------------------------
#if (LED_DRIVER_ISSI37XX == ENABLED)
#include "Led37XX_prv.h"
#include "Gpio.h"
#include "I2c.h"
#include "I2cMgr.h"
#include <string.h>

#ifndef LED37XX_IC_3736
    #error Must define LED37XX_IC_3736 in Led37XX_prv.h as ENABLED or DISABLED
#endif

#ifndef LED37XX_IC_3731
    #error Must define LED37XX_IC_3731 in Led37XX_prv.h as ENABLED or DISABLED
#endif

#ifndef LED37XX_IC_3732
    #error Must define LED37XX_IC_3732 in Led37XX_prv.h as ENABLED or DISABLED
#endif

#ifndef LED37XX_IC_3728
    #error Must define LED37XX_IC_3728 in Led37XX_prv.h as ENABLED or DISABLED
#endif


#if (LED37XX_IC_3736 == ENABLED)
    //! Hold the number of outputs of a single drive.
    #define     LED37XX_NUM_LEDS        96
    #define     LED37XX_CNTRL_NUM       LED37XX_NUM_LEDS/4

    #define     ADD_COMMAND_REGISTER               0xFD
    #define     ADD_COMMAND_WRIET_LOCK_REGISTER    0xFE

    typedef struct
    {
        unsigned char LedBuffer[LED37XX_CNTRL_NUM];   //00h - 17H
        unsigned char PWMBuffer[LED37XX_NUM_LEDS*2];  //00h - BEH
    } LED37XX_MEM_TYPE;

    typedef enum _LED37XX_STATE_TYPE
    {
        LED37XX_STATE_IDLE,
        LED3736_STATE_UNLOCK_REGISTER_COMMAND,
        LED3736_STATE_SWITCH_TO_FUNCTION_REGISTER,
        LED3736_STATE_SET_NORMAL_OPERATION,
        LED3736_STATE_UNLOCK_REGISTER_COMMAND2,
        LED3736_STATE_SWITCH_TO_LED_CONTROL_REGISTER,
        LED3736_STATE_WRITE_LED_STATUS,
        LED3736_STATE_UNLOCK_REGISTER_COMMAND3,
        LED3736_STATE_SWITCH_TO_PWM_REGISTER,
        LED3736_STATE_WRITE_PWM_STATUS,
        LED37XX_STATE_WAIT_WRITE_END,
        LED37XX_STATE_DELAY
    } LED37XX_STATE_TYPE;
#elif (LED37XX_IC_3731 == ENABLED || LED37XX_IC_3732 == ENABLED)
    //! Hold the number of outputs of a single drive.
    #define     LED37XX_NUM_LEDS        144
    #define     LED37XX_CNTRL_NUM       LED37XX_NUM_LEDS/8

    #define     ADD_COMMAND_REGISTER               0xFD

    typedef struct
    {
        unsigned char LedBuffer[LED37XX_CNTRL_NUM];   //00h - 11H
        unsigned char PWMBuffer[LED37XX_NUM_LEDS];  //24h - B3H
    } LED37XX_MEM_TYPE;

    typedef enum _LED37XX_STATE_TYPE
    {
        LED37XX_STATE_IDLE,
        LED3732_STATE_SELECT_SECTION_FRAME,
        LED3732_STATE_WRITE_LED_STATUS,
        LED3732_STATE_WRITE_PWM_STATUS,
        LED3732_STATE_SELECT_SECTION_COMMAND,
        LED3732_STATE_SET_DISPLAY_OPTION,
        LED3732_STATE_SET_NORMAL_OPERATION,
        LED3732_STATE_SET_GCCR_CURRENT,
		LED3732_STATE_SET_GHOST_IMAGE_PREVENTION,
        LED37XX_STATE_WAIT_WRITE_END,
        LED37XX_STATE_DELAY
    } LED37XX_STATE_TYPE;
#elif (LED37XX_IC_3728 == ENABLED)
    //! Hold the number of outputs of a single drive.
    #define     LED37XX_NUM_LEDS        64
    #define     LED37XX_CNTRL_NUM       LED37XX_NUM_LEDS/8

    typedef struct
    {
        uint8 LedBuffer[LED37XX_CNTRL_NUM];   //01h - 08H
    } LED37XX_MEM_TYPE;

    typedef enum _LED37XX_STATE_TYPE
    {
        LED37XX_STATE_IDLE,
//        LED3728_STATE_CONFIG,
        LED3728_STATE_WRITE_CFR,
        LED3728_STATE_WRITE_LER,
        LED3728_STATE_WRITE_AER,
        LED3728_STATE_WRITE_BUFFER,
        LED3728_STATE_UPDATE_COLUM,
        LED37XX_STATE_WAIT_WRITE_END,
        LED37XX_STATE_DELAY
    } LED37XX_STATE_TYPE;

    static uint8 Colum_Index;
#else
    #error No driver is defined in Led37XX_prv.h! Must define one of 3731/3732/3736/3728 as ENABLED.
#endif

//! I2C structure for device
typedef struct _LED37XX_DEVICE_TYPE
{
    I2C_ENUM_TYPE i2c_channel;
    uint8 i2c_address;
}LED37XX_DEVICE_TYPE;

#define LED37XX_MAX_PWM_VALUE   255

static const LED37XX_DEVICE_TYPE LED37XX_DEVICES[LED37XX_NUM_DEVICES] = LED37XX_DEVICE_LIST;
static uint8 Led37XX_DeviceHandle[LED37XX_NUM_DEVICES];
static uint8 Led37XX_DeviceIndex;
static LED37XX_STATE_TYPE Led37XX_State;
static LED37XX_MEM_TYPE Led37XX_Buffer[LED37XX_NUM_DEVICES];
static LED37XX_MEM_TYPE Led37XX_BackupBuffer[LED37XX_NUM_DEVICES];

static uint8 Data_Send_Buffer[2];
static uint8 Delay_Counter;
static uint16 Led37XX_TimeOut;
static uint16 Led37XX_ForceWriteTimout[LED37XX_NUM_DEVICES];

//-------------------------------------- PRIVATE (Function Prototypes) ------------------------------------------------
//=====================================================================================================================
//-------------------------------------- Public Functions -------------------------------------------------------------
//=====================================================================================================================

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      It Initializes the module Led37XX and its variables
 *
 */
void Led37XX__Initialize(void )
{
    uint8 index;

    Led37XX_State = LED37XX_STATE_IDLE;
    Led37XX_DeviceIndex = 0;
    memset(Led37XX_DeviceHandle, I2CMGR_HANDLE_INVALID, sizeof(Led37XX_DeviceHandle));
    memset(Led37XX_Buffer,0,sizeof(Led37XX_Buffer));
    memset(Led37XX_BackupBuffer,0,sizeof(Led37XX_BackupBuffer));
    memset(Data_Send_Buffer,0,sizeof(Data_Send_Buffer));
    memset(Led37XX_ForceWriteTimout,0,sizeof(Led37XX_ForceWriteTimout));

    Delay_Counter = DELAY_COUNT;
    Led37XX_TimeOut = LED37XX_TIMEOUT;

    for (index=0; index < LED37XX_NUM_DEVICES; index++)
    {
        Led37XX_DeviceHandle[index] = I2cMgr__GetDeviceHandle(LED37XX_DEVICES[index].i2c_channel);
    }

    Gpio__PinConfig(LED37XX_SDB_PORT,LED37XX_SDB_PORTBIT,OUTPUT_PUSHPULL);
    Gpio__PinWrite(LED37XX_SDB_PORT,LED37XX_SDB_PORTBIT,TRUE);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief This method handles the communication with the LED37XX driver over the I2c Bus
 * @details This Handler returns a value which can be used to safely multiple different devices on the I2c Bus
 * @return  LED37XX_STATUS_TYPE
 *      @retval LED37XX_STATUS_BUSY which means the process is in progress and can not be interrupted by another device in the I2c bus.
 *      @retval LED37XX_STATUS_FREE which means the previews process is finished with no errors and the I2c bus can be handle to another device on the bus.
 *      @retval LED37XX_STATUS_ERROR which means the previews process is finished with errors but the I2c bus can be handled to another device on the bus.
 */
LED37XX_STATUS_TYPE Led37XX__Handler(void )
{
    uint8 device_index;

    switch (Led37XX_State)
    {
        case LED37XX_STATE_IDLE:
            for(device_index = 0; device_index < LED37XX_NUM_DEVICES; device_index++)
            {
                Led37XX_DeviceIndex = device_index;

                Led37XX_ForceWriteTimout[Led37XX_DeviceIndex]++;

                if(Led37XX_ForceWriteTimout[Led37XX_DeviceIndex] > LED37XX_FORCE_WRITE_TIMEOUT )
                {
                    Led37XX_ForceWriteTimout[Led37XX_DeviceIndex] = 0;

                    if(I2cMgr__RequestBus(Led37XX_DeviceHandle[Led37XX_DeviceIndex]) == TRUE)
                    {
                        // update the data buffer
                        memcpy(&Led37XX_BackupBuffer[Led37XX_DeviceIndex], &Led37XX_Buffer[Led37XX_DeviceIndex], sizeof(LED37XX_MEM_TYPE));

                        I2c__Initialize(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, LED37XX_I2C_SPEED, I2C_ADDR_7BITS, LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_address);

                        Led37XX_State++;
                        Led37XX_TimeOut = LED37XX_TIMEOUT;
                        break;
                    }
                }
                else
                {
                    // if there is data changed for the considered device.
                    // Any change on  LED will be detected resulting in a write procedure to that device.
                    if (memcmp(&Led37XX_Buffer[Led37XX_DeviceIndex], &Led37XX_BackupBuffer[Led37XX_DeviceIndex], sizeof(LED37XX_MEM_TYPE)) != 0)
                    {
                        if(I2cMgr__RequestBus(Led37XX_DeviceHandle[Led37XX_DeviceIndex]) == TRUE)
                        {
                            // update the data buffer
                            memcpy(&Led37XX_BackupBuffer[Led37XX_DeviceIndex], &Led37XX_Buffer[Led37XX_DeviceIndex], sizeof(LED37XX_MEM_TYPE));

                            I2c__Initialize(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, LED37XX_I2C_SPEED, I2C_ADDR_7BITS, LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_address);

                            Led37XX_State++;
                            Led37XX_TimeOut = LED37XX_TIMEOUT;
                            break;
                        }
                    }
                }
            }
            break;

#if (LED37XX_IC_3736 == ENABLED)
        case LED3736_STATE_UNLOCK_REGISTER_COMMAND:
            if (I2c__GetStatus(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                // Unlock Command Register -> 0xFE-0xC5
                Data_Send_Buffer[0] = 0xC5;
                I2c__Write(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, ADD_COMMAND_WRIET_LOCK_REGISTER, Data_Send_Buffer, 1);
                Led37XX_State = LED3736_STATE_SWITCH_TO_FUNCTION_REGISTER;
                Led37XX_TimeOut = LED37XX_TIMEOUT;
            }
            break;

        case LED3736_STATE_SWITCH_TO_FUNCTION_REGISTER:
            if (I2c__GetStatus(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                // Unlock Command Register -> 0xFD-0x03
                Data_Send_Buffer[0] = 0x03;
                I2c__Write(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, ADD_COMMAND_REGISTER, Data_Send_Buffer, 1);
                Led37XX_State = LED3736_STATE_SET_NORMAL_OPERATION;
                Led37XX_TimeOut = LED37XX_TIMEOUT;
            }
            break;

        case LED3736_STATE_SET_NORMAL_OPERATION:
            if (I2c__GetStatus(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                Data_Send_Buffer[0] = 0x01;   //B_EN=0,PWM MODE ENABLE; SSD=1,NORMAL OPERATE
                Data_Send_Buffer[1] = 0xFF;   //GCCR=0xFF,GLOBAL CURRENT
                I2c__Write(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, 0x00, Data_Send_Buffer, 2);
                Led37XX_State = LED3736_STATE_UNLOCK_REGISTER_COMMAND2;
                Led37XX_TimeOut = LED37XX_TIMEOUT;
            }
            break;

        case LED3736_STATE_UNLOCK_REGISTER_COMMAND2:
            if (I2c__GetStatus(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                // Unlock Command Register -> 0xFE-0xC5
                Data_Send_Buffer[0] = 0xC5;
                I2c__Write(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, ADD_COMMAND_WRIET_LOCK_REGISTER, Data_Send_Buffer, 1);
                Led37XX_State = LED3736_STATE_SWITCH_TO_PWM_REGISTER;
                Led37XX_TimeOut = LED37XX_TIMEOUT;
            }
            break;

        case LED3736_STATE_SWITCH_TO_PWM_REGISTER:
            if (I2c__GetStatus(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                // Unlock Command Register -> 0xFD-0x01
                Data_Send_Buffer[0] = 0x01;
                I2c__Write(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, ADD_COMMAND_REGISTER, Data_Send_Buffer, 1);
                Led37XX_State = LED3736_STATE_WRITE_PWM_STATUS;
                Led37XX_TimeOut = LED37XX_TIMEOUT;
            }
            break;

        case LED3736_STATE_WRITE_PWM_STATUS:
            if (I2c__GetStatus(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                // perform Write to PWM registers
                I2c__Write(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, 0x00, Led37XX_BackupBuffer[Led37XX_DeviceIndex].PWMBuffer, LED37XX_NUM_LEDS*2);
                Led37XX_State = LED3736_STATE_UNLOCK_REGISTER_COMMAND3;
                Led37XX_TimeOut = LED37XX_TIMEOUT;
            }
            break;

        case LED3736_STATE_UNLOCK_REGISTER_COMMAND3:
            if (I2c__GetStatus(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                // Unlock Command Register -> 0xFE-0xC5
                Data_Send_Buffer[0] = 0xC5;
                I2c__Write(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, ADD_COMMAND_WRIET_LOCK_REGISTER, Data_Send_Buffer, 1);
                Led37XX_State = LED3736_STATE_SWITCH_TO_LED_CONTROL_REGISTER;
                Led37XX_TimeOut = LED37XX_TIMEOUT;
            }
            break;

        case LED3736_STATE_SWITCH_TO_LED_CONTROL_REGISTER:
            if (I2c__GetStatus(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                // Unlock Command Register -> 0xFD-0x00
                Data_Send_Buffer[0] = 0x00;
                I2c__Write(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, ADD_COMMAND_REGISTER, Data_Send_Buffer, 1);
                Led37XX_State = LED3736_STATE_WRITE_LED_STATUS;
                Led37XX_TimeOut = LED37XX_TIMEOUT;
            }
            break;

        case LED3736_STATE_WRITE_LED_STATUS:
            if (I2c__GetStatus(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                // perform write to LED state register
                I2c__Write(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, 0x00, Led37XX_BackupBuffer[Led37XX_DeviceIndex].LedBuffer, LED37XX_CNTRL_NUM);
                Led37XX_State = LED37XX_STATE_WAIT_WRITE_END;
                Led37XX_TimeOut = LED37XX_TIMEOUT;
            }
            break;
#endif

#if (LED37XX_IC_3731 == ENABLED || LED37XX_IC_3732 == ENABLED)
        case LED3732_STATE_SELECT_SECTION_FRAME:
            if (I2c__GetStatus(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                // Frame Selection -> 0XFD-0X00
                Data_Send_Buffer[0] = 0x00;
                I2c__Write(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, ADD_COMMAND_REGISTER, Data_Send_Buffer, 1);
                Led37XX_State = LED3732_STATE_WRITE_PWM_STATUS;
                Led37XX_TimeOut = LED37XX_TIMEOUT;
            }
            break;

        case LED3732_STATE_WRITE_PWM_STATUS:
            if (I2c__GetStatus(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                // perform Write to PWM registers
                I2c__Write(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, 0x24, Led37XX_BackupBuffer[Led37XX_DeviceIndex].PWMBuffer, LED37XX_NUM_LEDS);
                Led37XX_State = LED3732_STATE_WRITE_LED_STATUS;
                Led37XX_TimeOut = LED37XX_TIMEOUT;
            }
            break;

        case LED3732_STATE_WRITE_LED_STATUS:
            if (I2c__GetStatus(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                // perform write to LED state register
                I2c__Write(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, 0x00, Led37XX_BackupBuffer[Led37XX_DeviceIndex].LedBuffer, LED37XX_CNTRL_NUM);
                Led37XX_State = LED3732_STATE_SELECT_SECTION_COMMAND;
                Led37XX_TimeOut = LED37XX_TIMEOUT;
            }
            break;
        case LED3732_STATE_SELECT_SECTION_COMMAND:
            if (I2c__GetStatus(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                // Frame Selection -> 0XFD-0X0B
                Data_Send_Buffer[0] = 0x0B;
                I2c__Write(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, ADD_COMMAND_REGISTER, Data_Send_Buffer, 1);
                Led37XX_State = LED3732_STATE_SET_DISPLAY_OPTION;
                Led37XX_TimeOut = LED37XX_TIMEOUT;
            }
            break;

        case LED3732_STATE_SET_DISPLAY_OPTION:
            if (I2c__GetStatus(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                // Set Display Option register  -> Intensity Control 0x05-0x00
                Data_Send_Buffer[0] = 0x00;
                I2c__Write(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, 0x05, Data_Send_Buffer, 1);
                Led37XX_State = LED3732_STATE_SET_NORMAL_OPERATION;
                Led37XX_TimeOut = LED37XX_TIMEOUT;
            }
            break;

        case LED3732_STATE_SET_NORMAL_OPERATION:
            if (I2c__GetStatus(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                Data_Send_Buffer[0] = 1;
                I2c__Write(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, 0x0A, Data_Send_Buffer, 1);
                Led37XX_State = LED3732_STATE_SET_GCCR_CURRENT;
                Led37XX_TimeOut = LED37XX_TIMEOUT;
            }
            break;

        case LED3732_STATE_SET_GCCR_CURRENT:
            if (I2c__GetStatus(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                Data_Send_Buffer[0] = 0xFF; //Current Settings
                I2c__Write(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, 0x04, Data_Send_Buffer, 1);
                Led37XX_State = LED3732_STATE_SET_GHOST_IMAGE_PREVENTION;
                Led37XX_TimeOut = LED37XX_TIMEOUT;
            }
            break;
        case LED3732_STATE_SET_GHOST_IMAGE_PREVENTION:
        	if(I2c__GetStatus(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
        	{
        		Data_Send_Buffer[0] = 0x10; //Ghost Image prevention Settings
        		I2c__Write(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, 0xC2, Data_Send_Buffer, 1);
        		Led37XX_State = LED37XX_STATE_WAIT_WRITE_END;
        		Led37XX_TimeOut = LED37XX_TIMEOUT;
        	}
        	break;
#endif

#if (LED37XX_IC_3728 == ENABLED)
        case LED3728_STATE_WRITE_CFR:
            if (I2c__GetStatus(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                // to force 8x8 matrix mode -> 00H - 0x00
                Data_Send_Buffer[0] = 0x00;
                // perform the write
                I2c__Write(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, 0x00, Data_Send_Buffer, 1);
                Led37XX_State = LED3728_STATE_WRITE_LER;
                Led37XX_TimeOut = LED37XX_TIMEOUT;
            }
            break;

        case LED3728_STATE_WRITE_LER:
            if (I2c__GetStatus(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                // to force 40ma in lightling effect register -> 0DH - 0x1c
                Data_Send_Buffer[0] = 0x1C;
                // perform the write
                I2c__Write(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, 0x0d, Data_Send_Buffer, 1);
                Led37XX_State = LED3728_STATE_WRITE_AER;
                Led37XX_TimeOut = LED37XX_TIMEOUT;
            }
            break;

        case LED3728_STATE_WRITE_AER:
            if (I2c__GetStatus(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                Data_Send_Buffer[0] = 0x00;
                // perform the write
                I2c__Write(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, 0x0f, Data_Send_Buffer, 1);
                Led37XX_State = LED3728_STATE_WRITE_BUFFER;
                Led37XX_TimeOut = LED37XX_TIMEOUT;
                Colum_Index = 0;
            }
            break;

        case LED3728_STATE_WRITE_BUFFER:
            if (I2c__GetStatus(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                // perform the write
                I2c__Write(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, Colum_Index + 1, &Led37XX_BackupBuffer[Led37XX_DeviceIndex].LedBuffer[Colum_Index], 1);
                Colum_Index ++;
                if (Colum_Index > 7)
                {
                    Led37XX_State = LED3728_STATE_UPDATE_COLUM;
                }
                Led37XX_TimeOut = LED37XX_TIMEOUT;
            }
            break;

        case LED3728_STATE_UPDATE_COLUM:
            if (I2c__GetStatus(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                // to force update colum register -> 0CH - 0x00
                Data_Send_Buffer[0] = 0x00;
                // perform the write
                I2c__Write(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, 0x0C, Data_Send_Buffer, 1);
                Led37XX_State = LED37XX_STATE_WAIT_WRITE_END;
                Led37XX_TimeOut = LED37XX_TIMEOUT;
            }
            break;
#endif

        case LED37XX_STATE_WAIT_WRITE_END:
            if (I2c__GetStatus(LED37XX_DEVICES[Led37XX_DeviceIndex].i2c_channel, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                I2cMgr__ReleaseBus(Led37XX_DeviceHandle[Led37XX_DeviceIndex]);
                // when finished go back to idle state
                Led37XX_State = LED37XX_STATE_DELAY;
                Delay_Counter = DELAY_COUNT;
                Led37XX_TimeOut = LED37XX_TIMEOUT;
            }
            break;

        case LED37XX_STATE_DELAY:
            Delay_Counter--;
            if (Delay_Counter == 0)
            {
                Led37XX_State = LED37XX_STATE_IDLE;
            }
            break;

        default:
            Led37XX_State = LED37XX_STATE_IDLE;
            break;
    }

    if ((Led37XX_State == LED37XX_STATE_IDLE) || (Led37XX_State == LED37XX_STATE_DELAY))
    {
        return (LED37XX_STATUS_FREE);
    }
    else
    {
        if (Led37XX_TimeOut)
        {
            Led37XX_TimeOut--;
            return (LED37XX_STATUS_BUSY);
        }
        else
        {
            Led37XX_State = LED37XX_STATE_IDLE;
            return (LED37XX_STATUS_ERROR);
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief This method is used to set the value for one of the 2 internal PWMs
 * @param device    define which device is begin considered
 * @param led       define the specific output of the selected device.
 * @param counts    define the duty-cycle for the defined pwm.
 */
void Led37XX__SetIntensity(uint8 device, uint8 led, uint8 intensity)
{
    intensity = (int) ((intensity * LED37XX_MAX_PWM_VALUE) / 100);

    if ((device < LED37XX_NUM_DEVICES) && (led < LED37XX_NUM_LEDS))
    {
        #if (LED37XX_IC_3736 == ENABLED)
            Led37XX_Buffer[device].PWMBuffer[led*2] = intensity;
        #endif
        #if (LED37XX_IC_3731 == ENABLED || LED37XX_IC_3732 == ENABLED)
            Led37XX_Buffer[device].PWMBuffer[led] = intensity;
        #endif
        #if (LED37XX_IC_3728 == ENABLED)
        #endif
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief This method connected a specific output with one of the possible options.
 * @param device define which device is begin considered
 * @param led    define the specific output of the selected device.
 * @param state define where the output should be connected to (LED3736_LED_OFF, LED3736_LED_ON)
 *
 */
void Led37XX__SetLed(uint8 device, uint8 led, LED37XX_LED_STATE state)
{
    uint8 aux_i;
    uint8 array_count_i;
    uint8 bit_cir_i;

    if ((device < LED37XX_NUM_DEVICES) && (led < LED37XX_NUM_LEDS) && (state <= LED37XX_LED_ON))
    {
        #if (LED37XX_IC_3736 == ENABLED)
            if (led > 0)
            {
                array_count_i = led / 4;
                aux_i = led % 4;
                aux_i = aux_i << 1;
            }
            else
            {
                // Avoid divided by 0
                array_count_i = 0;
                aux_i = 0;
            }

            bit_cir_i = 0x01; // setup to rotate bit

            if (aux_i > 0)
            {
                bit_cir_i = 0x01 << aux_i;
            }

            if (state == LED37XX_LED_OFF)
            {
                bit_cir_i = ~bit_cir_i;
                Led37XX_Buffer[device].LedBuffer[array_count_i] &= bit_cir_i;
            }
            else
            {
                Led37XX_Buffer[device].LedBuffer[array_count_i] |= bit_cir_i;
            }
        #endif
        #if (LED37XX_IC_3731 == ENABLED || LED37XX_IC_3732 == ENABLED || LED37XX_IC_3728 == ENABLED)
            if (led > 0)
            {
                array_count_i = led / 8;
                aux_i = led % 8;
            }
            else
            {
                // Avoid divided by 0
                array_count_i = 0;
                aux_i = 0;
            }

            bit_cir_i = 0x01; // setup to rotate bit

            if (aux_i > 0)
            {
                bit_cir_i = 0x01 << aux_i;
            }

            if (state == LED37XX_LED_OFF)
            {
                bit_cir_i = ~bit_cir_i;
                Led37XX_Buffer[device].LedBuffer[array_count_i] &= bit_cir_i;
            }
            else
            {
                Led37XX_Buffer[device].LedBuffer[array_count_i] |= bit_cir_i;
            }
        #endif
    }
}

#endif // (LED_DRIVER_ISSI37XX == ENABLED)
