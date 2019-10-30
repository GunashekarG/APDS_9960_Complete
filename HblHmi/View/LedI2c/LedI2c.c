/**
 *  @file       
 *
 *  @brief     Interface layer to interpret requests to turn LEDs on/off for LEDs controlled by the 9532 IC.
 *
 *  @details    
 *
 *  $Header: $
 *
 *  @copyright  Copyright 2015-$Date$. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------

//-------------------------------------- Include Files ----------------------------------------------------------------
#include "SystemConfig.h"
#include "LedI2c.h"

#if (LED_DRIVER_CAT9532 == ENABLED)
#include "Led9532.h"
#endif //(LED_DRIVER_CAT9532 == ENABLED)

#if (LED_DRIVER_PCA9952 == ENABLED)
#include "Led9952.h"
#endif //(LED_DRIVER_PCA9952 == ENABLED)

#if (LED_DRIVER_ISSI37XX == ENABLED)
#include "Led37XX.h"
#endif //(LED_DRIVER_ISSI37XX == ENABLED)

//-------------------------------------- PRIVATE (Variables, Constants & Defines) -------------------------------------
//-------------------------------------- PRIVATE (Function Prototypes) ------------------------------------------------
#if (LED_DRIVER_CAT9532 == ENABLED)
void LedI2cCAT9532(uint8 device_index, uint8 led_index, uint8 intensity, LED9532_PWM_TYPE pwm_index);
#endif
#if (LED_DRIVER_PCA9952 == ENABLED)
void LedI2cPCA9952(uint8 device_index, uint8 led_index, uint8 intensity);
#endif
#if (LED_DRIVER_ISSI37XX == ENABLED)
void LedI2cISSI37XX(uint8 device_index, uint8 led_index, uint8 intensity);
#endif

//=====================================================================================================================
//-------------------------------------- Public Functions -------------------------------------------------------------
//=====================================================================================================================

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief      Sets a specific physical LED.
 *
 * @param device_index : index of the 9532 device on which the LED resides
 * @param led_index: LED on the 9532 device to control
 * @param intensity: perentage intensity to apply to the LED (0-100)
 * @param pwm_index: the pwm index to use (if needed)
 */
#if (LED_DRIVER_CAT9532 == ENABLED || LED_DRIVER_PCA9952 == ENABLED || LED_DRIVER_ISSI37XX == ENABLED)

void LedI2c__SetLed(LEDI2C_DRIVER_TYPE driver_type, uint8 device_index, uint8 led_index, uint8 intensity, uint8 pwm_index)
{
    switch(driver_type)
    {
#if(LED_DRIVER_CAT9532 == ENABLED)
        case LEDI2C_DRIVER_CAT9532:
            LedI2cCAT9532(device_index, led_index, intensity, (LED9532_PWM_TYPE) pwm_index);
            break;
#endif
#if (LED_DRIVER_PCA9952 == ENABLED)
        case LEDI2C_DRIVER_PCA9952:
            LedI2cPCA9952(device_index, led_index, intensity);
            break;
#endif
#if (LED_DRIVER_ISSI37XX == ENABLED)
        case LEDI2C_DRIVER_ISSI37XX:
            LedI2cISSI37XX(device_index, led_index, intensity);
            break;
#endif
    }
}

#endif // (LED_DRIVER_PCA9952 == ENABLED)
//=====================================================================================================================
//-------------------------------------- Private Functions ------------------------------------------------------------
//=====================================================================================================================
#if (LED_DRIVER_CAT9532 == ENABLED)
void LedI2cCAT9532(uint8 device_index, uint8 led_index, uint8 intensity, LED9532_PWM_TYPE pwm_index)
{
    LED9532_LED_TYPE led_output;

    switch(intensity)
    {
        case 0:
            led_output = LED9532_LED_OFF;
            break;
        case 100:
            led_output = LED9532_LED_ON;
            break;
        default:
            if (pwm_index < NUM_LED9532_PWM)
            {
                Led9532__SetIntensity(device_index, pwm_index, intensity);
                if (pwm_index == LED9532_PWM0)
                {
                    led_output = LED9532_LED_PWM0;
                }
                else if (pwm_index == LED9532_PWM1)
                {
                    led_output = LED9532_LED_PWM1;
                }
            }
            break;
    }
    Led9532__SetLed(device_index, led_index, led_output);
}
#endif

#if (LED_DRIVER_PCA9952 == ENABLED)
void LedI2cPCA9952(uint8 device_index, uint8 led_index, uint8 intensity)
{
    LED9952_LED_STATE led_output;

    switch(intensity)
    {
        case 0:
            led_output = LED9952_LED_STATE_OFF;
            break;
        case 100:
            led_output = LED9952_LED_STATE_ON;
            break;
        default:
            Led9952__SetIntensity(device_index, led_index, intensity);
            led_output = LED9952_LED_STATE_PWM_GROUP;
            break;
    }
    Led9952__SetLed(device_index, led_index, led_output);
}
#endif

#if (LED_DRIVER_ISSI37XX == ENABLED)
void LedI2cISSI37XX(uint8 device_index, uint8 led_index, uint8 intensity)
{
    Led37XX__SetIntensity(device_index, led_index, intensity);

    if(intensity == 0)
    {
        Led37XX__SetLed(device_index, led_index, LED37XX_LED_OFF);
    }
    else
    {
        Led37XX__SetLed(device_index, led_index, LED37XX_LED_ON);
    }
}
#endif
