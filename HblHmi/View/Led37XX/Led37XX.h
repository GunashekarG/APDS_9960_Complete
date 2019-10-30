/**
 *  @file
 *
 *  @brief      Standard API for ISSI 37XX LED Driver over I2c
 *
 *  @copyright  Copyright 2015-$Date: 2017/03/20 16:00:00EST $. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
#ifndef LED37XX_H_
#define LED37XX_H_

#if (LED_DRIVER_ISSI37XX == ENABLED)
//=====================================================================================================================
//-------------------------------------- PUBLIC (Extern Variables, Constants & Defines) -------------------------------
//=====================================================================================================================

//! I2C state for 9952 drivers
typedef enum _LED37XX_STATUS_TYPE
{
    LED37XX_STATUS_BUSY,
    LED37XX_STATUS_FREE,
    LED37XX_STATUS_ERROR
} LED37XX_STATUS_TYPE;

//! LED output types
typedef enum _LED37XX_LED_STATE
{
    LED37XX_LED_OFF = 0,
    LED37XX_LED_ON  = 1
} LED37XX_LED_STATE;

//=====================================================================================================================
//-------------------------------------- PUBLIC (Function Prototypes) -------------------------------------------------
//=====================================================================================================================
void Led37XX__Initialize(void);
LED37XX_STATUS_TYPE Led37XX__Handler(void);
void Led37XX__SetIntensity(uint8 device, uint8 led, uint8 intensity);
void Led37XX__SetLed(uint8 device, uint8 led, LED37XX_LED_STATE state);

#endif // (LED_DRIVER_ISSI37XX == ENABLED)
#endif // LED37XX_H_


