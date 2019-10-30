/**
 *  @file
 *
 *  @brief      Module provides interface to turn LEDs on/off.
 *
 *  @copyright  Copyright 2015-$Date: 2015/08/12 08:50:22EDT $. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------

//-------------------------------------- Include Files ----------------------------------------------------------------
#include "SystemConfig.h"
#include "LedMgr.h"
#include "LedMgr_prv.h"

#ifndef LED_DRIVER_CAT9532
    #error Must define LED_DRIVER_CAT9532 in SystemConfig.h as ENABLED or DISABLED
#endif

#ifndef LED_DRIVER_EXTERNAL
    #error Must define LED_DRIVER_EXTERNAL in SystemConfig.h as ENABLED or DISABLED
#endif

#ifndef LED_DRIVER_MATRIX
    #error Must define LED_DRIVER_MATRIX in SystemConfig.h as ENABLED or DISABLED
#endif

#ifndef LED_DRIVER_PCA9952
    #error Must define LED_DRIVER_PCA9952 in SystemConfig.h as ENABLED or DISABLED
#endif

#ifndef LED_DRIVER_ISSI37XX
    #error Must define LED_DRIVER_ISSI37XX in SystemConfig.h as ENABLED or DISABLED
#endif

#ifndef LED_DIRECT_WITH_PWM_FEATURE
    #define LED_DIRECT_WITH_PWM_FEATURE DISABLED
#endif

#ifndef LEDMGR_USE_LED_STATE_BUFFERING
    #define LEDMGR_USE_LED_STATE_BUFFERING DISABLED
#endif


#if (LED_DRIVER_EXTERNAL == ENABLED)
    #include "API012Exp.h"
#endif
#include "Gpio.h"
#if (LED_DIRECT_WITH_PWM_FEATURE == ENABLED)
    #include "LedDirectPwm.h"
#endif
#if (LED_DRIVER_CAT9532 == ENABLED ||\
     LED_DRIVER_PCA9952 == ENABLED ||\
     LED_DRIVER_ISSI37XX == ENABLED)
    #include "LedI2c.h"
#endif
#if (LED_DRIVER_CAT9532 == ENABLED)
    #include "Led9532.h"
#endif
#if (LED_DRIVER_PCA9952 == ENABLED)
    #include "Led9952.h"
#endif
#if (LED_DRIVER_ISSI37XX == ENABLED)
    #include "Led37XX.h"
#endif
#if (LED_DRIVER_MATRIX == ENABLED)
    #include "LedMatrix.h"
#endif
#include "SettingFile.h"
#include <string.h>
#include "VirtualPin.h"

//-------------------------------------- PRIVATE (Variables, Constants & Defines) -------------------------------------

typedef struct
{
    uint8       Intensity;
    uint8       Pwm_Index;
}LEDMGR_LED_STATE_REQUEST;

#if (LED_DRIVER_EXTERNAL == ENABLED)
    static uint8 External_Led_Buffer[NUM_OF_EXTERNAL_LEDS];    //! Desired state of the LEDs on the HMI expansion board(s)
#endif

static ON_OFF_TYPE Led_States[NUM_OF_LEDS];
static LEDMGR_LED_STATE_REQUEST Led_State_Request[NUM_OF_LEDS];
static uint8 Num_Of_Groups;
//! Holds the number of LEDs configured in GESE. 
//! LedMgr supports up to NUM_OF_LEDS. However, application might have configured fewer.
static uint8 Num_of_LEDs;

//! Pointer to table that defines a configuration for each LED
static const LEDMGR_CONFIG_TABLE_TYPE* Led_Ptr;

//-------------------------------------- PRIVATE (Function Prototypes) ------------------------------------------------
static void ApplyPatternToGroup(uint16 group_index, uint32 pattern_word, uint8 intensity, uint8 pwm_index);
static void RequestLedState(uint16 led_index, uint8 intensity, uint8 pwm_index);

//=====================================================================================================================
//-------------------------------------- Public Functions -------------------------------------------------------------
//=====================================================================================================================

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      This initializes all animations
 */
void LedMgr__Initialize(void)
{
    uint8 index;
    SETTINGFILE_LOADER_TYPE sf_loader;

    memset(Led_States, 0, sizeof(Led_States));
    memset(Led_State_Request, 0, sizeof(Led_State_Request));


    Num_Of_Groups = SettingFile__GetNumDisplacements(0, SF_PTR_UI_LED_GROUP_CONFIGURATION); //The setting file index is not currently used
    Led_Ptr = NULL;

    if (SettingFile__BasicLoader(SF_PTR_UI_IO_CONFIG, SF_DISPL_LED_CONFIGURATION, &sf_loader) == PASS)
    {
        //Initialize Led_Ptr for all modules within LedMgr
        Led_Ptr = (LEDMGR_CONFIG_TABLE_TYPE*) sf_loader.Data;   //lint !e927

        Num_of_LEDs = sf_loader.Length/sizeof(LEDMGR_CONFIG_TABLE_TYPE);

        // If there are any LEDs driven directly through GPIO, configure the GPIO pin
        for(index = 0; index < Num_of_LEDs; index++)
        {
            if (Led_Ptr[index].Driver_Type == LEDMGR_DRIVER_DIRECT)
            {
                Gpio__PinConfig(VIRTUALPIN_TABLE[Led_Ptr[index].Parameter_1].port,
                                VIRTUALPIN_TABLE[Led_Ptr[index].Parameter_1].pin,
                                VIRTUALPIN_TABLE[Led_Ptr[index].Parameter_1].pin_config);
            }
        }
    }

    // Initialize modules related to the different LED drivers
    #if (LED_DIRECT_WITH_PWM_FEATURE == ENABLED)
        LedDirectPwm__Initialize();
    #endif
    #if (LED_DRIVER_MATRIX == ENABLED)
        LedMatrix__Initialize();
    #endif
    #if (LED_DRIVER_CAT9532 == ENABLED)
        Led9532__Initialize();
    #endif
    #if (LED_DRIVER_PCA9952 == ENABLED)
        Led9952__Initialize();
    #endif
    #if (LED_DRIVER_ISSI37XX == ENABLED)
        Led37XX__Initialize();
    #endif
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief	Returns the number of LEDs the application has configured in GESE
 * @return
 */
uint16 LedMgr__GetNumOfLeds(void)
{
    return(Num_of_LEDs);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief   Returns how many LEDs the application has configured in GESE using the specified driver
 * @param   driver_type
 * @return
 */
uint16 LedMgr__GetNumOfLedsByDriver(LEDMGR_DRIVER_TYPE driver_type)
{
    uint8 index;
    uint16 leds_count = 0;

    if(Led_Ptr != NULL)
    {
        for (index = 0; index < Num_of_LEDs; index++)
        {
            if (Led_Ptr[index].Driver_Type == driver_type)
            {
                leds_count++;
            }
        }
    }

    return(leds_count);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      This function will get the led_index of the led at the specified position in a group
 *
 *  @param      group_index : The group to look in
 *  @param      member_index : The index within the group to get the led_index of
 *
 *  @return     returns INVALID_LED_INDEX if the index is outside of the group or the group_index is not valid
 *                      otherwise the enum value of an individual led
 */
uint16 LedMgr__GetLedInGroup(uint16 group_index, uint8 member_index)
{
    uint16 retval = INVALID_LED_INDEX;
    SETTINGFILE_LOADER_TYPE sf_loader;

    //Verify that the group is in the valid range
    if ((group_index < Num_Of_Groups) &&
        (SettingFile__BasicLoader(SF_PTR_UI_LED_GROUP_CONFIGURATION, group_index, &sf_loader) == PASS))
    {
            retval = sf_loader.Data[member_index];
    }

    return(retval);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *
 * @param led_index
 * @return
 */
ON_OFF_TYPE LedMgr__GetLedState(uint16 led_index)
{
    return(Led_States[led_index]);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *
 * @param group_index
 * @param led_index
 * @return
 */
ON_OFF_TYPE LedMgr__GetLedStateInGroup(uint16 group_index, uint8 member_index)
{
    return(Led_States[LedMgr__GetLedInGroup(group_index, member_index)]);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Sets a request for a single led to the specified intensity.
 *
 *  @param      led_index : the index of the led to set
 *  @param      intensity : the intensity to apply
 */
void LedMgr__SetLed(uint16 led_index, uint8 intensity, uint8 pwm_index)
{
    if(led_index < Num_of_LEDs)
    {
        if (intensity > 100)
        {
            intensity = 100;
        }
        RequestLedState(led_index, intensity, pwm_index);

        #if (LEDMGR_USE_LED_STATE_BUFFERING != ENABLED)
        // Set new LED state immediately
        LedMgr__SetLedState(led_index);
        #endif
    }
    //else, no action necessary for LED's
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Sets all leds to the specified intensity
 *
 *  @param      intensity : the intensity to apply
 */
void LedMgr__SetAllLeds(uint8 intensity, uint8 pwm_index)
{
    uint8 index;

    for (index = 0; index < Num_of_LEDs; index++)
    {
        LedMgr__SetLed(index, intensity, pwm_index);
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Sets all leds within a group using a pattern, leds are either set to active_intensity or turned off.
 *  @details    If the group has more led's than the max group size, additional leds will be set to OFF, the size of
 *              groups can be increased in LedMgr_prm.h.
 *
 *  @param      group_index : The group to set
 *  @param      pattern_word : A bit pattern in which each bit describes the state of an led within a group.
 *  @param      intensity : The intensity to apply to leds that are turned ON by the pattern
 */
void LedMgr__SetPatternInGroup(uint16 group_index, uint32 pattern_word, uint8 intensity, uint8 pwm_index)
{
    ApplyPatternToGroup(group_index, pattern_word, intensity, pwm_index);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Sets all leds within a group using a pattern, leds are either set to active_intensity or turned off.
 *  @details    If the group has more led's than the max group size, additional leds will be set to OFF, the size of
 *              groups can be increased in LedMgr_prm.h.
 *
 *  @param      group_index : The group to set
 *  @param      start: 0 to (LED_MAX_GROUP_SIZE - 1)
 *  @param      end: 1 to LED_MAX_GROUP_SIZE
 *  @param      intensity : The intensity to apply to leds that are within the range of start-end
 */
void LedMgr__SetRangeInGroup(uint16 group_index, uint8 start, uint8 end, uint8 intensity, uint8 pwm_index)
{
    ApplyPatternToGroup(group_index, RANGE_TO_PATTERN(start, end), intensity, pwm_index);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Sets all leds within a group
 *  @details    If the group has more led's than the max group size, additional leds will be set to OFF, the size of
 *              groups can be increased in LedMgr_prm.h.
 *
 *  @param      group_index : The group to set
 *  @param      intensity : The intensity to apply to leds in the group
 */
void LedMgr__SetGroup(uint16 group_index, uint8 intensity, uint8 pwm_index)
{
    ApplyPatternToGroup(group_index, PATTERN_ALL, intensity, pwm_index);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Sets a single led within a group, all other members are turned off
 *  @details    If the group has more led's than the max group size, additional leds will be set to OFF, the size of
 *              groups can be increased in LedMgr_prm.h.
 *
 *  @param      group_index : The group to set
 *  @param      member : the index of the member within the group to set to intensity
 *  @param      intensity : The intensity to apply to leds in the group
 */
void LedMgr__SetLedInGroup(uint16 group_index, uint8 member, uint8 intensity, uint8 pwm_index)
{
    ApplyPatternToGroup(group_index, PATTERN_BIT_SELECTED(member), intensity, pwm_index);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Sets an led and all led's above it within a group, all other members are turned off
 *  @details    If the group has more led's than the max group size, additional leds will be set to OFF, the size of
 *              groups can be increased in LedMgr_prm.h.
 *
 *  @param      group_index : The group to set
 *  @param      member : the index of the member within the group which is the first led to turn on
 *  @param      intensity : The intensity to apply to leds in the group
 */
void LedMgr__SetGroupFromMemberUp(uint16 group_index, uint8 member, uint8 intensity, uint8 pwm_index)
{
    ApplyPatternToGroup(group_index, ((PATTERN_ALL >> member) << member), intensity, pwm_index);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Sets an led and all led's below it within a group, all other members are turned off
 *  @details    If the group has more led's than the max group size, additional leds will be set to OFF, the size of
 *              groups can be increased in LedMgr_prm.h.
 *
 *  @param      group_index : The group to set
 *  @param      member : the index of the member within the group which is the led led to turn on
 *  @param      intensity : The intensity to apply to leds in the group
 */
void LedMgr__SetGroupFromMemberDown(uint16 group_index, uint8 member, uint8 intensity, uint8 pwm_index)
{
    member++;
    ApplyPatternToGroup(group_index, (~((PATTERN_ALL >> member) << member)), intensity, pwm_index);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      Sets an individual led without modifying other led's within the group
 *  @details    This function may modify led's beyond the maximum group size.
 *
 *  @param      group_index : The group to set
 *  @param      member : the index of the member within the group which will be set to intensity
 *  @param      intensity : The intensity to apply to the modified led
 */
void LedMgr__ModifyLedInGroup(uint16 group_index, uint8 member, uint8 intensity, uint8 pwm_index)
{
    LedMgr__SetLed(LedMgr__GetLedInGroup(group_index, member), intensity, pwm_index);
}
//---------------------------------------------------------------------------------------------------------------------
/**
 *    @brief 	Gets amount of LEDs inside the given led group
 *    @param 	led_group
 *    @return	Returns 0xFF if the group index is out of range or the setting file data cannot be loaded, else it
 *              returns the number of Led's in the group
 */
uint8 LedMgr__GetLedNumInGroup(uint16 led_group)
{
    uint8 retval = INVALID_LED_GROUP_SIZE;
	SETTINGFILE_LOADER_TYPE sf_loader;

    //Verify that the group is in the valid range
    if ((led_group < Num_Of_Groups) &&
        (SettingFile__BasicLoader(SF_PTR_UI_LED_GROUP_CONFIGURATION, led_group, &sf_loader) == PASS))
    {
        retval = sf_loader.Length;
        if(sf_loader.Data[sf_loader.Length-1] == 0xFF)
        {
            retval--;
        }
    }

    return(retval);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief   Sets the state of the LED using request buffer
 * @param   led_index : the index of the led to set
 */
void LedMgr__SetLedState(uint16 led_index)
{
#if (LED_DIRECT_WITH_PWM_FEATURE == DISABLED)
    uint8 intensity;
#endif
	if(led_index < Num_of_LEDs)
	{
        if (Led_State_Request[led_index].Intensity > 0)
        {
            Led_States[led_index] = ON;
        }
        else
        {
            Led_States[led_index] = OFF;
        }

        // Check to ensure the led pointer contains valid data and that the index is within range
        if (Led_Ptr != NULL)
        {
            switch(Led_Ptr[led_index].Driver_Type)
            {
                case LEDMGR_DRIVER_MATRIX:
                    #if (LED_DRIVER_MATRIX == ENABLED)
                        if (Led_State_Request[led_index].Intensity > 0)
                        {
                            Led_State_Request[led_index].Intensity = 1; // Matrix only supports 0/1 values
                        }

                        LedMatrix__SetLed((Led_Ptr[led_index].Parameter_1), (Led_Ptr[led_index].Parameter_2), (BOOL_TYPE)Led_State_Request[led_index].Intensity);
                    #endif
                    break;
                case LEDMGR_DRIVER_DIRECT:
                    #if (LED_DIRECT_WITH_PWM_FEATURE == ENABLED)
                         LedDirectPwm__SetLed(Led_Ptr[led_index].Parameter_1, Led_State_Request[led_index].Intensity, Led_Ptr[led_index].Parameter_2);
                    #else
                        if (Led_State_Request[led_index].Intensity > 0)
                        {
                            Led_State_Request[led_index].Intensity = 1; // Direct LED only supports 0/1 values
                        }
                        intensity = Led_State_Request[led_index].Intensity;
						//Check for Inverted Logic config
                        if(Led_Ptr[led_index].Parameter_2 != FALSE)
                        {
                            intensity = 1 - intensity;
                        }
                        Gpio__PinWrite(VIRTUALPIN_TABLE[Led_Ptr[led_index].Parameter_1].port, VIRTUALPIN_TABLE[Led_Ptr[led_index].Parameter_1].pin, (BOOL_TYPE)intensity);
                    #endif
                    break;
                case LEDMGR_DRIVER_CAT9532:
                    #if (LED_DRIVER_CAT9532 == ENABLED)
                        LedI2c__SetLed(LEDI2C_DRIVER_CAT9532, Led_Ptr[led_index].Parameter_1, Led_Ptr[led_index].Parameter_2, Led_State_Request[led_index].Intensity, Led_State_Request[led_index].Pwm_Index);
                    #endif
                    break;
                case LEDMGR_DRIVER_EXTERNAL:
                    #if (LED_DRIVER_EXTERNAL == ENABLED)
                        if (Led_State_Request[led_index].Intensity > 0)
                        {
                            Led_State_Request[led_index].Intensity = 1;
                        }
                        External_Led_Buffer[Led_Ptr[led_index].Parameter_2] = Led_State_Request[led_index].Intensity;
                        //Note: Currently API012Exp module does not support multiple HMI expansion boards (expansion HMI node ID hardcoded to 5)
                        API012Exp__UpdateLeds(External_Led_Buffer, sizeof(External_Led_Buffer));
                    #endif
                    break;
                case LEDMGR_DRIVER_PCA9952:
                    #if (LED_DRIVER_PCA9952 == ENABLED)
                        LedI2c__SetLed(LEDI2C_DRIVER_PCA9952, Led_Ptr[led_index].Parameter_1, Led_Ptr[led_index].Parameter_2, Led_State_Request[led_index].Intensity, Led_State_Request[led_index].Pwm_Index);
                    #endif
                    break;
                case LEDMGR_DRIVER_ISSI37XX:
                    #if (LED_DRIVER_ISSI37XX == ENABLED)
                        LedI2c__SetLed(LEDI2C_DRIVER_ISSI37XX, Led_Ptr[led_index].Parameter_1, Led_Ptr[led_index].Parameter_2, Led_State_Request[led_index].Intensity, Led_State_Request[led_index].Pwm_Index);
                    #endif
                    break;
                case LEDMGR_DRIVER_CUSTOM:
                    #ifdef LEDMGR_CUSTOM_DRIVER_SET_LED
                        LEDMGR_CUSTOM_DRIVER_SET_LED(led_index, Led_State_Request[led_index].Intensity, Led_State_Request[led_index].Pwm_Index, Led_Ptr[led_index].Parameter_1, Led_Ptr[led_index].Parameter_2, Led_Ptr[led_index].Parameter_3);
                    #endif
                    break;
                default:
                    break;
            }
        }
	}
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief Get the settigFile configuration for a given LED
 * @param led_index
 * @return settingFile parameters
 */
LEDMGR_CONFIG_TABLE_TYPE* LedMgr__GetLedConfig(uint16 led_index)
{
    LEDMGR_CONFIG_TABLE_TYPE* cfg_ptr = NULL;

    if( (Led_Ptr != NULL) &&
        (led_index < Num_of_LEDs))
    {
        cfg_ptr = (LEDMGR_CONFIG_TABLE_TYPE*)(&Led_Ptr[led_index]); //lint !e929
    }

    return(cfg_ptr);
}

//=====================================================================================================================
//-------------------------------------- Private Functions ------------------------------------------------------------
//=====================================================================================================================

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief   This function applies a pattern to a group defined by global variables
 * @param   group_index : The group to set
 * @param   pattern_word : A bit pattern in which each bit describes the state of an led within a group.
 * @param   intensity : The intensity to apply to leds that are turned ON by the pattern
 */
static void ApplyPatternToGroup(uint16 group_index, uint32 pattern_word, uint8 intensity, uint8 pwm_index)
{
    uint8 index;
    SETTINGFILE_LOADER_TYPE sf_loader;

    if (SettingFile__BasicLoader(SF_PTR_UI_LED_GROUP_CONFIGURATION, group_index, &sf_loader) == PASS)
    {
        for (index = 0; index < sf_loader.Length; index++)
        {
            if (sf_loader.Data[index] != INVALID_LED_INDEX)  // Cover case where an extra led index was added for word alignment only
            {
                // If the corresponding pattern bit is 1, set the led to intensity
                if ((pattern_word & 1) == 1)
                {
                    LedMgr__SetLed(sf_loader.Data[index], intensity, pwm_index);
                }
                else // Else if the bit is 0, or the index is beyond the pattern size, set the led to off.
                {
                    LedMgr__SetLed(sf_loader.Data[index], 0, pwm_index);
                }
                // Shift the next bit into place to be checked
                pattern_word = (pattern_word >> 1);
            }
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief   Sets the request for the LED state based on the LED driver
 * @param   led_index : the index of the led to set
 * @param   intensity : the intensity to apply in percentage (0-100)
 * @param   pwm_index : the pwm index to use
 */
static void RequestLedState(uint16 led_index, uint8 intensity, uint8 pwm_index)
{
    if(led_index < Num_of_LEDs)
    {
        Led_State_Request[led_index].Intensity = intensity;
        Led_State_Request[led_index].Pwm_Index = pwm_index;
    }
}

