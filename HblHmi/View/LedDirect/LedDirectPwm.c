/**
 *  @file       
 *
 *  @brief      Module to control intensity levels on a Direct LED (an LED connected to a pin on the micro).
 *
 *  @copyright  Copyright 2017-$Date$. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------

//-------------------------------------- Include Files ----------------------------------------------------------------
#include "SystemConfig.h"
#include "LedDirectPwm.h"

#ifndef LED_DIRECT_WITH_PWM_FEATURE
    #define LED_DIRECT_WITH_PWM_FEATURE DISABLED
#endif

#if (LED_DIRECT_WITH_PWM_FEATURE == ENABLED)
#include "LedDirectPwm_prv.h"

#include "Gpio.h"
#include "VirtualPin.h"

//-------------------------------------- PRIVATE (Variables, Constants & Defines) -------------------------------------

//! Structure for direct LED variables
typedef struct
{
    uint8 Led_Intensity;
    uint8 Virtual_Pin;
    BOOL_TYPE Inverted_Logic;
}LEDDIRECT_TYPE;

//! Holds the Direct LED request
static LEDDIRECT_TYPE LedDirect_Request[NUM_OF_LED_DIRECT];

//! Used to control the LED intensity in the background loop
static uint8 Current_Intensity;

//! Maximum intensity level in percentage
#define LED_MAX_INTENSITY   (100)

//! Definition used to indicate an unused entry in the LedDirect_Request array
#define LED_DIRECT_UNUSED   (0xFF)

//! Keeps track of number of LEDs that are in use in the LedDirect_Request array
static uint8 LedDirect_Allocation;

//-------------------------------------- PRIVATE (Function Prototypes) ------------------------------------------------

//=====================================================================================================================
//-------------------------------------- Public Functions -------------------------------------------------------------
//=====================================================================================================================

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      It Initializes the module and its variables
 */
void LedDirectPwm__Initialize(void)
{
    for(uint8 index=0; index < NUM_OF_LED_DIRECT; index++)
    {
        LedDirect_Request[index].Led_Intensity = 0;
        LedDirect_Request[index].Virtual_Pin   = LED_DIRECT_UNUSED;
    }

    Current_Intensity = 0;
    LedDirect_Allocation = 0;
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Manages the direct LEDs that have intensity greater than zero or less than 100
 *  @details    This can be called from ViewMgr or SRMain_prv.h
 */
void LedDirectPwm__Handler1ms(void)
{
    uint8 index;
    ON_OFF_TYPE led_on_off;

    // Update current intensity level
    Current_Intensity = (Current_Intensity + LED_DIRECT_PWM_INTENSITY_INTERVAL);

    // Reset intensity level if it has overflowed
    if(Current_Intensity > LED_MAX_INTENSITY)
    {
        Current_Intensity = LED_DIRECT_PWM_INTENSITY_INTERVAL;
    }

    // Turn on/off all LEDs that are supposed to be controlled with an intensity level
    for(index=0; index < NUM_OF_LED_DIRECT; index++)
    {
        if(LedDirect_Request[index].Virtual_Pin != LED_DIRECT_UNUSED)
        {
            if (LedDirect_Request[index].Led_Intensity >= Current_Intensity)
            {
                led_on_off = ON;
            }
            else
            {
                led_on_off = OFF;
            }
            if(LedDirect_Request[index].Inverted_Logic != FALSE)
            {
                led_on_off = (ON_OFF_TYPE)(ON - led_on_off);
            }
            Gpio__PinWrite(VIRTUALPIN_TABLE[LedDirect_Request[index].Virtual_Pin].port, VIRTUALPIN_TABLE[LedDirect_Request[index].Virtual_Pin].pin, (BOOL_TYPE)led_on_off);
        }
        else
        {
            break;  //at the first unused position move out the loop
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief   Interface to use to set the state of a direct LED.
 * @param   virtual_pin: index in the virtual pin table that has the info for the pin that controls the LED
 * @param   intensity: intensity level for the LED in percentage (0-100)
 * @param   inverted: indicates if the the Direct LED is driven with inverted logic
 */
void LedDirectPwm__SetLed(uint8 virtual_pin, uint8 intensity, uint8 inverted)
{
    uint8 index;
    BOOL_TYPE pin_allocated = FALSE;

    //Look if already allocated
    for(index=0; index < LedDirect_Allocation; index++)
    {
        if(LedDirect_Request[index].Virtual_Pin == virtual_pin)
        {
            pin_allocated = TRUE;
            break;
        }
    }

    // If not allocated, attempt to allocate an index in the array
    if(pin_allocated == FALSE)
    {
        if(LedDirect_Allocation < NUM_OF_LED_DIRECT)
        {
            LedDirect_Request[LedDirect_Allocation].Virtual_Pin = virtual_pin;
            LedDirect_Request[LedDirect_Allocation].Led_Intensity = intensity;
            LedDirect_Request[LedDirect_Allocation].Inverted_Logic = (BOOL_TYPE)inverted;
            LedDirect_Allocation++;   //move to next spot (or the max limit)
        }
    }
    else
    {
        LedDirect_Request[index].Led_Intensity = intensity;
    }
}

//=====================================================================================================================
//-------------------------------------- Private Functions ------------------------------------------------------------
//=====================================================================================================================

#endif // (LED_DIRECT_WITH_PWM_FEATURE == ENABLED)
