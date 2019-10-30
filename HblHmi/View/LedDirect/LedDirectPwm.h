/**
 *  @file       
 *
 *  @brief      Public interface for the LedDirectPwm module.
 *
 *  @copyright  Copyright 2017-$Date$. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
#ifndef LEDDIRECTPWM_H_
#define LEDDIRECTPWM_H_

#if (LED_DIRECT_WITH_PWM_FEATURE == ENABLED)

//=====================================================================================================================
//-------------------------------------- PUBLIC (Extern Variables, Constants & Defines) -------------------------------
//=====================================================================================================================

//=====================================================================================================================
//-------------------------------------- PUBLIC (Function Prototypes) -------------------------------------------------
//=====================================================================================================================
void LedDirectPwm__Initialize(void);
void LedDirectPwm__Handler1ms(void);
void LedDirectPwm__SetLed(uint8 virtual_pin, uint8 intensity, uint8 inverted);

#endif // (LED_DIRECT_WITH_PWM_FEATURE == ENABLED)

#endif // LEDDIRECTPWM_H_
