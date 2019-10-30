/**
 *  @file
 *
 *  @brief      Public interface for the Led9952 module.
 *
 *  @copyright  Copyright 2016-$Date: 2016/07/26 10:57:03EST $. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
#ifndef LED9952_H_
#define LED9952_H_

#if (LED_DRIVER_PCA9952 == ENABLED)
//=====================================================================================================================
//-------------------------------------- PUBLIC (Extern Variables, Constants & Defines) -------------------------------
//=====================================================================================================================

//! I2C state for 9952 drivers
typedef enum _LED9952_STATUS_TYPE
{
    LED9952_STATUS_BUSY,
    LED9952_STATUS_FREE,
    LED9952_STATUS_ERROR
} LED9952_STATUS_TYPE;

//! LED output types
typedef enum _LED9952_LED_STATE
{
    LED9952_LED_STATE_OFF            = 0x0, //!< LED driver x is off (default power-up state).
    LED9952_LED_STATE_ON             = 0x1, //!< LED driver x is fully on (individual brightness and group dimming/blinking not controlled).
    LED9952_LED_STATE_PWM_INDIVIDUAL = 0x2, //!< LED driver x individual brightness can be controlled through its PWMx register.
    LED9952_LED_STATE_PWM_GROUP      = 0x3  //!< LED driver x individual brightness and group dimming/blinking can be controlled through its PWMx register and the GRPPWM registers.
} LED9952_LED_STATE;

//=====================================================================================================================
//-------------------------------------- PUBLIC (Function Prototypes) -------------------------------------------------
//=====================================================================================================================
void Led9952__Initialize(void);
LED9952_STATUS_TYPE Led9952__Handler(void);
void Led9952__SetIntensity(uint8 device, uint8 led, uint8 intensity);
void Led9952__SetGroupIntensity(uint8 device, uint8 intensity);
void Led9952__SetCurrentGain(uint8 device, uint8 led, uint8 intensity);
void Led9952__SetLed(uint8 device, uint8 led, LED9952_LED_STATE state);

#endif // (LED_DRIVER_PCA9952 == ENABLED)
#endif // LED9952_H_
