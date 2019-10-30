/**
 *  @file       
 *
 *  @brief      Public interface for the LedI2c module.
 *
 *  @copyright  Copyright 2015-$Date$. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
#ifndef LEDI2C_H_
#define LEDI2C_H_

#if (LED_DRIVER_CAT9532 == ENABLED || LED_DRIVER_PCA9952 == ENABLED || LED_DRIVER_ISSI37XX == ENABLED)

//=====================================================================================================================
//-------------------------------------- PUBLIC (Extern Variables, Constants & Defines) -------------------------------
//=====================================================================================================================
typedef enum
{
    LEDI2C_DRIVER_CAT9532 = 0,
    LEDI2C_DRIVER_PCA9952,
    LEDI2C_DRIVER_ISSI37XX
}LEDI2C_DRIVER_TYPE;

//=====================================================================================================================
//-------------------------------------- PUBLIC (Function Prototypes) -------------------------------------------------
//=====================================================================================================================
void LedI2c__SetLed(LEDI2C_DRIVER_TYPE driver_type, uint8 device_index, uint8 led_index, uint8 intensity, uint8 pwm_index);

#endif // (LED_DRIVER_CAT9532 == ENABLED || LED_DRIVER_PCA9952 == ENABLED || LED_DRIVER_ISSI37XX == ENABLED)
#endif // LEDI2C_H_
