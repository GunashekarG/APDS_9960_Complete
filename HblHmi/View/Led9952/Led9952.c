/**
 *  @file
 *
 *  @brief      This module handles the PCA9952 and PCA9955 Led drivers.
 *
 *  @details    Each LEDn output can be off, on (no PWM control), set at its individual PWM controller value or at both individual and group PWM controller values.
 *
 *              The PCA9952 is identical to PCA9955 except for the following differences:
 *                  - The PCA9952 has only three hardware address pins compared to four on PCA9955.
 *                  - The PCA9952 has an output enable pin (OE) and the PCA9955 does not.
 *
 *  @copyright  Copyright 2015-$Date: 2016/01/15 10:57:07EST $. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------

//-------------------------------------- Include Files ----------------------------------------------------------------
#include "SystemConfig.h"
#include "Led9952.h"

#if (LED_DRIVER_PCA9952 == ENABLED)
#include "Led9952_prv.h"
#include "I2c.h"
#include "I2cMgr.h"
#include <string.h>

//-------------------------------------- PRIVATE (Variables, Constants & Defines) -------------------------------------

//! Hold the number of outputs of a single drive.
#define LED9952_NUM_LEDS 16

//! I2C structure for device
typedef struct
{
    I2C_ENUM_TYPE i2c_channel;
    uint8 i2c_address;
}LED9952_DEVICE_TYPE;

//! LED9952 states
typedef enum _LED9952_STATE_TYPE
{
    LED9952_STATE_IDLE,
    LED9952_STATE_PWM_WRITE,
    LED9952_STATE_CONFIG_WRITE,
    LED9952_STATE_IREF_WRITE,
    LED9952_STATE_WAIT_WRITE_END
} LED9952_STATE_TYPE;

//! Positions of LEDOUTx register
typedef enum
{
   LED_POSITION_0 = 0,
   LED_POSITION_1,
   LED_POSITION_2,
   LED_POSITION_3
}LED9952_POSITION_TYPE;

//! Definition of the positions of LEDOUTx register
typedef struct
{
    LED9952_LED_STATE Position_0   :2; //!< Holds the output method for the LDR0 to LDR3 of the register
    LED9952_LED_STATE Position_1   :2; //!< Holds the output method for the LDR4 to LDR7 of the register
    LED9952_LED_STATE Position_2   :2; //!< Holds the output method for the LDR8 to LDR11 of the register
    LED9952_LED_STATE Position_3   :2; //!< Holds the output method for the LDR12 to LDR15 of the register
}LED9952_LEDOUT_TYPE;

//! Configuration buffer
typedef struct
{
    uint8 Mode[2];
    LED9952_LEDOUT_TYPE Ledout[4];
} LED9952_CONFIG_MEM_TYPE;

//! PWM buffer
typedef struct
{
    uint8 Grppwm;
    uint8 Grpfreq;
    uint8 Pwm[16];
} LED9952_MEM_TYPE;

//! Current gain buffer
typedef struct
{
    uint8 Iref[16];
} LED9952_MEM_IREF_TYPE;

static const LED9952_DEVICE_TYPE LED9952_DEVICES[LED9952_NUM_DEVICES] = LED9952_DEVICE_LIST;

static LED9952_STATE_TYPE Led9952_State;
static uint8 Led9952_Device_Index;

static LED9952_CONFIG_MEM_TYPE Led9952_Config_Buffer[LED9952_NUM_DEVICES];
static BOOL_TYPE Led9952_Config_Update[LED9952_NUM_DEVICES];


static LED9952_MEM_IREF_TYPE Led9952_Iref_Buffer[LED9952_NUM_DEVICES];
static BOOL_TYPE Led9952_Iref_Update[LED9952_NUM_DEVICES];

static LED9952_MEM_TYPE Led9952_Pwm_Buffer[LED9952_NUM_DEVICES];
static BOOL_TYPE Led9952_Pwm_Update[LED9952_NUM_DEVICES];

static uint8 Led9952_TimeOut;

static uint8 Led9952_DeviceHandle[LED9952_NUM_DEVICES];

//-------------------------------------- PRIVATE (Function Prototypes) ------------------------------------------------

//=====================================================================================================================
//-------------------------------------- Public Functions -------------------------------------------------------------
//=====================================================================================================================

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      It Initializes the module Led9952 and its variables
 *
 */
void Led9952__Initialize(void)
{
    uint8 index;
    Led9952_State = LED9952_STATE_IDLE;
    Led9952_Device_Index = 0;
    memset(Led9952_Pwm_Buffer,0,sizeof(Led9952_Pwm_Buffer));
    memset(Led9952_Config_Buffer,0,sizeof(Led9952_Config_Buffer));
    memset(Led9952_Iref_Buffer,LED9952_INIT_IREF_VALUE,sizeof(Led9952_Iref_Buffer));
    memset(Led9952_DeviceHandle, I2CMGR_HANDLE_INVALID, sizeof(Led9952_DeviceHandle));

    Led9952_TimeOut = 0;

    for (index=0; index < LED9952_NUM_DEVICES; index++)
    {
        Led9952_Iref_Update[index] = TRUE;
        Led9952_Config_Update[index] = TRUE;
        Led9952_Pwm_Update[index] = TRUE;

        Led9952_Config_Buffer[index].Mode[0] = 0xC0;      // Auto-Increment for MODE1 to IREF15 control registers (00h to 31h).
        Led9952_Config_Buffer[index].Mode[1] = 0x08;      // Outputs change on ACK

        Led9952_Pwm_Buffer[index].Grppwm = 0xFF;          // Set the master PWM register to maximum duty cycle
        Led9952_Pwm_Buffer[index].Grpfreq = 0xFF;         // Set the master frequency register to maximum duty cycle

        Led9952_DeviceHandle[index] = I2cMgr__GetDeviceHandle(LED9952_DEVICES[index].i2c_channel);
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief Led9952 Handler which it is recommended to be called every 1ms
 * @return Handler status LED9952_STATUS_TYPE
 */
LED9952_STATUS_TYPE Led9952__Handler(void)
{
    uint8 device_index;

    switch (Led9952_State)
    {
        case LED9952_STATE_IDLE:
            for(device_index = 0; device_index < LED9952_NUM_DEVICES; device_index++)
            {
                Led9952_Device_Index = device_index;

                if (Led9952_Iref_Update[Led9952_Device_Index] == TRUE )
                {
                    if(I2cMgr__RequestBus(Led9952_DeviceHandle[Led9952_Device_Index]) == TRUE)
                    {
                        I2c__Initialize(LED9952_DEVICES[Led9952_Device_Index].i2c_channel,LED9952_I2C_SPEED,I2C_ADDR_7BITS,LED9952_DEVICES[Led9952_Device_Index].i2c_address);
                        Led9952_State = LED9952_STATE_IREF_WRITE;
                        Led9952_TimeOut = LED9952_TIMEOUT;
                        break;
                    }
                }
                else if (Led9952_Pwm_Update[Led9952_Device_Index] == TRUE)
                {
                    if(I2cMgr__RequestBus(Led9952_DeviceHandle[Led9952_Device_Index]) == TRUE)
                    {
                        I2c__Initialize(LED9952_DEVICES[Led9952_Device_Index].i2c_channel,LED9952_I2C_SPEED,I2C_ADDR_7BITS,LED9952_DEVICES[Led9952_Device_Index].i2c_address);
                        Led9952_State = LED9952_STATE_PWM_WRITE;
                        Led9952_TimeOut = LED9952_TIMEOUT;
                        break;
                    }
                }
                else if (Led9952_Config_Update[Led9952_Device_Index] == TRUE)
                {
                    if(I2cMgr__RequestBus(Led9952_DeviceHandle[Led9952_Device_Index]) == TRUE)
                    {
                        I2c__Initialize(LED9952_DEVICES[Led9952_Device_Index].i2c_channel,LED9952_I2C_SPEED,I2C_ADDR_7BITS,LED9952_DEVICES[Led9952_Device_Index].i2c_address);
                        Led9952_State = LED9952_STATE_CONFIG_WRITE;
                        Led9952_TimeOut = LED9952_TIMEOUT;
                        break;
                    }
                }
            }

            break;
        case LED9952_STATE_PWM_WRITE:
            if (I2c__GetStatus(LED9952_DEVICES[Led9952_Device_Index].i2c_channel,I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                I2c__Write(LED9952_DEVICES[Led9952_Device_Index].i2c_channel,LED9952_ADDR_PWM,(uint8 *)&Led9952_Pwm_Buffer[Led9952_Device_Index],sizeof(LED9952_MEM_TYPE));
                Led9952_Pwm_Update[Led9952_Device_Index] = FALSE;
                Led9952_State = LED9952_STATE_WAIT_WRITE_END;
            }
            break;
        case LED9952_STATE_CONFIG_WRITE:
            if (I2c__GetStatus(LED9952_DEVICES[Led9952_Device_Index].i2c_channel,I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                I2c__Write(LED9952_DEVICES[Led9952_Device_Index].i2c_channel,LED9952_ADDR_CONFIG,(uint8 *)&Led9952_Config_Buffer[Led9952_Device_Index],sizeof(LED9952_CONFIG_MEM_TYPE));
                Led9952_Config_Update[Led9952_Device_Index] = FALSE;
                Led9952_State = LED9952_STATE_WAIT_WRITE_END;
            }
            break;
        case LED9952_STATE_IREF_WRITE:
            if (I2c__GetStatus(LED9952_DEVICES[Led9952_Device_Index].i2c_channel,I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                I2c__Write(LED9952_DEVICES[Led9952_Device_Index].i2c_channel,LED9952_ADDR_IREF,(uint8 *)&Led9952_Iref_Buffer[Led9952_Device_Index],sizeof(LED9952_MEM_IREF_TYPE));
                Led9952_Iref_Update[Led9952_Device_Index] = FALSE;
                Led9952_State = LED9952_STATE_WAIT_WRITE_END;
            }
            break;
        case LED9952_STATE_WAIT_WRITE_END:
            if (I2c__GetStatus(LED9952_DEVICES[Led9952_Device_Index].i2c_channel,I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                I2cMgr__ReleaseBus(Led9952_DeviceHandle[Led9952_Device_Index]);
                Led9952_State = LED9952_STATE_IDLE;
            }
            break;
        default:
            break;
    }

    if (Led9952_State == LED9952_STATE_IDLE)
    {
        return (LED9952_STATUS_FREE);
    }
    else
    {
        if (Led9952_TimeOut)
        {
            Led9952_TimeOut--;
            return (LED9952_STATUS_BUSY);
        }
        else
        {
            Led9952_State = LED9952_STATE_IDLE;
            return (LED9952_STATUS_ERROR);
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief This method is used to set the intensity of each individual LED
 *
 * @param device --> related to the device in the list of devices configured in the prv file.
 * @param led    --> defines the specific LED in the specific device from 0 to 15
 * @param intensity --> define the duty-cycle percentage
 *
 * @note This method just adjust the intensity of the leds on the PWM registers. To link the output use the Led9552__SetLed() method.
 */
void Led9952__SetIntensity(uint8 device, uint8 led, uint8 intensity)
{
    if ((device < LED9952_NUM_DEVICES)&&
        (led < LED9952_NUM_LEDS))
    {
            Led9952_Pwm_Buffer[device].Pwm[led] = LED9952_PWM_COUNT_TABLE[intensity * (sizeof(LED9952_PWM_COUNT_TABLE)-1) / 100];        // Adjust pwm counts based on number of pwm selections;
            Led9952_Pwm_Update[device] = TRUE;
    }
}
//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief This method is used to set the intensity of a LED group.
 *
 * @param device --> related to the device in the list of devices configured in the prv file.
 * @param intensity --> define the duty-cycle percentage
 *
 * @note  A LED group is specify by setting the LEDs as LED9952_LED_PWM_GROUP.
 */
void Led9952__SetGroupIntensity(uint8 device, uint8 intensity)
{
    if (device < LED9952_NUM_DEVICES)
    {
        Led9952_Pwm_Buffer[device].Grppwm = LED9952_PWM_COUNT_TABLE[intensity * (sizeof(LED9952_PWM_COUNT_TABLE)-1) / 100];        // Adjust pwm counts based on number of pwm selections;;
        Led9952_Pwm_Update[device] = TRUE;
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief This method is used to set the current can of each LED.
 *
 * @details The current gain is used to get a more specific color at RBG application. It's also possible Control the duty cycle and the current gain.
 *
 * @param device --> related to the device in the list of devices configured in the prv file.
 * @param led    --> defines the specific LED in the specific device from 0 to 15
 * @param max_intensity --> define the maximum visual intensity percentage (controls max current output)
 *
 * @note There's no option to link the current gain to the out put. The user can keep the LED set as LED9952_LED_ON or LED9952_LED_PWM_INDIVIDUAL.
 */
void Led9952__SetCurrentGain(uint8 device, uint8 led, uint8 max_intensity)
{
    max_intensity = (int) ((max_intensity * LED9952_MAX_IREF_VALUE) / 100);
    if ((device < LED9952_NUM_DEVICES)&&
        (led < LED9952_NUM_LEDS ))
    {
        Led9952_Iref_Buffer[device].Iref[led] = max_intensity;
        Led9952_Iref_Update[device] = TRUE;
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief This method connected a specific output with one of the possible options.
 * @param device --> define which device is begin considered
 * @param led    --> define the specific output of the selected device.
 * @param state  --> define where the output should be connected to (LED9952_LED_OFF, LED9952_LED_ON, LED9952_LED_PWM_INDIVIDUAL or  LED9952_LED_PWM_GROUP)
 *
 * @note The output options have the following characteristics : \n
 *
 * @note LED9952_LED_OFF           : LED driver x is off (default power-up state). \n
 * @note LED9952_LED_ON            : LED driver x is fully on (individual brightness and group dimming/blinking not controlled). \n
 * @note LED9952_LED_PWM_INDIVIDUAL: LED driver x individual brightness can be controlled through its PWMx register. \n
 * @note LED9952_LED_PWM_GROUP     : LED driver x individual brightness and group dimming/blinking can be controlled through its PWMx register and the GRPPWM registers. \n
 *
 */
void Led9952__SetLed(uint8 device, uint8 led, LED9952_LED_STATE state)
{
    uint8 led_register;
    uint8 led_symbol;
    LED9952_LED_STATE old_state;

    if ((device < LED9952_NUM_DEVICES)&&
        (led < LED9952_NUM_LEDS ))
    {

        led_register = led / 4;
        led_symbol = led % 4;

        switch ((LED9952_POSITION_TYPE)led_symbol)
        {
            case LED_POSITION_0:
                old_state = Led9952_Config_Buffer[device].Ledout[led_register].Position_0;
                Led9952_Config_Buffer[device].Ledout[led_register].Position_0 = state;
                break;
            case LED_POSITION_1:
                old_state = Led9952_Config_Buffer[device].Ledout[led_register].Position_1;
                Led9952_Config_Buffer[device].Ledout[led_register].Position_1 = state;
                break;
            case LED_POSITION_2:
                old_state = Led9952_Config_Buffer[device].Ledout[led_register].Position_2;
                Led9952_Config_Buffer[device].Ledout[led_register].Position_2 = state;
                break;
            case LED_POSITION_3:
                old_state = Led9952_Config_Buffer[device].Ledout[led_register].Position_3;
                Led9952_Config_Buffer[device].Ledout[led_register].Position_3 = state;
                break;

            default:
                break;
        }

        if(old_state != state)
        {
            Led9952_Config_Update[device] = TRUE;
        }
    }
}

//=====================================================================================================================
//-------------------------------------- Private Functions ------------------------------------------------------------
//=====================================================================================================================

#endif // (LED_DRIVER_PCA9952 == ENABLED)
