/**
 *  @file
 *
 *  @brief      Public interface for the HblHmi module.
 *
 *  $Header: HblHmi.h 1.12 2015/08/17 13:05:11EDT Ryan P Kujawski (KUJAWRP) Exp  $
 *
 *  @copyright  Copyright 2015-2018: 2015/08/17 13:05:11EDT $. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
#ifndef HBLHMI_H_
#define HBLHMI_H_

#include "HblHmi_prm.h"
#include "HblDefs.h"
#include "SoundDefs.h"
#include "ViewMgr.h"

//=====================================================================================================================
//-------------------------------------- PUBLIC (Extern Variables, Constants & Defines) -------------------------------
//=====================================================================================================================

//! LEDs Priority index used to "check-out" the usage of an LED mask
typedef enum
{
    HBLHMI_LED_PRIORITY_INTERPRETER = 0,
    HBLHMI_LED_PRIORITY_SYSTEM,
    HBLHMI_LED_PRIORITY_EMBEDDED,
    HBLHMI_LED_PRIORITY_LEDANIMATIONMGR,
    HBLHMI_LED_PRIORITY_MAX
}HBLHMI_LED_PRIORITY_LEVEL_TYPE;

#ifndef HBLHMI_LED_PRIORITY_FEATURE
    //! When enabled the LED priority feature uses a priority level to determine when LEDs can be controlled
    //! See HBLHMI_LED_PRIORITY_LEVEL_TYPE for the list of priority levels
    #define HBLHMI_LED_PRIORITY_FEATURE ENABLED
#endif

//=====================================================================================================================
//-------------------------------------- PUBLIC (Function Prototypes) -------------------------------------------------
//=====================================================================================================================
void HblHmi__Initialize(void);
void HblHmi__MainHandler(void);
void HblHmi__AsynchHandler(void);
void HblHmi__Handler1ms(void);
void HblHmi__Handler(void);
void HblHmi__RegisterGIEventHandler(HBLHMI_GI_EVENT_HANDLER_TYPE event_handler);
void HblHmi__UnregisterGIEventHandler(HBLHMI_GI_EVENT_HANDLER_TYPE event_handler);
void HblHmi__RegisterSequenceHandler(HBLHMI_GI_EVENT_HANDLER_TYPE event_handler);
void HblHmi__UnregisterSequenceHandler(HBLHMI_GI_EVENT_HANDLER_TYPE event_handler);

//-------------------------------------------------------------------------------------------------
// Inputs Handling
//-------------------------------------------------------------------------------------------------

// LLI

HBL_LLI_ENUM_TYPE HblHmi__GetLLIIDByIndex(uint8 input);
uint8 HblHmi__GetLLIIndexByID(HBL_LLI_ENUM_TYPE lli_id, uint8 position);
void * HblHmi__GetLLIDataByIndex(uint8 index);
uint8 HblHmi__GetNumLLI(void);
uint8 HblHmi__GetLLISequenceIDByIndex(uint8 index);
uint8 HblHmi__GetLLIDataSizeByIndex(uint8 index);
uint8 HblHmi__GetLLIPositionByIndex(uint8 index);
HBL_LLI_ENUM_TYPE HblHmi__GetLLIIDByGIIndex(uint8 index);
uint8 HblHmi__GetLLIPositionByGIIndex(uint8 index);

// GI
void * HblHmi__GetGIByIndex(uint8 index);
void * HblHmi__GetGIData(HBLHMI_GI_READ_TYPE read_type, uint8 gi_position);
uint8 HblHmi__GetGIDataSizeByIndex(uint8 index);
HBLHMI_GI_ENUM_TYPE HblHmi__GetGIIDByIndex(uint8 input);
uint8 HblHmi__GetGIReadTypeByIndex(uint8 index);
uint8 HblHmi__GetGIPositionByIndex(uint8 index);
uint8 HblHmi__GetGIIndexByGIID(HBLHMI_GI_ENUM_TYPE input);
uint8 HblHmi__GetGIConfigIndexByType(HBLHMI_GI_READ_TYPE read_type, uint8 position);
HBLHMI_GI_TYPE * HblHmi__GetGITypeByGidata(HBLHMI_GI_READ_TYPE read_type, uint8 gi_position);
uint8 HblHmi__GetNumGI(void);
uint8 HblHmi__GetNumGIReadType(HBLHMI_GI_READ_TYPE read_type);
void HblHmi__SetGI(HBLHMI_GI_READ_TYPE read_type, uint8 position, uint8 value);
uint8 HblHmi__GetGISequenceIDByIndex(uint8 index);
void HblHmi__SetVirtualGiBuffer(HBLHMI_GI_READ_TYPE read_type, uint8 gi_position, uint8 val_gi_virtual);

//-------------------------------------------------------------------------------------------------
// Interface to the View
//-------------------------------------------------------------------------------------------------
void HblHmi__ClearView(void);
void HblHmi__DisplayFault(uint8 f_code, uint8 e_code, uint8 eng_code);
void HblHmi__PlaySoundComplex(uint8 sound_index, SOUND_VOLUME_TYPE volume_index, SOUND_PRIORITY_TYPE sound_priority, BOOL_TYPE can_interrupt);
void HblHmi__PlaySoundSimple(uint8 sound_index, SOUND_VOLUME_TYPE volume_index);
void HblHmi__SetObject(VISUAL_OBJECT_TYPE object_type, uint16 object_index, uint32 object_value, uint8 object_intensity, HBLHMI_LED_PRIORITY_LEVEL_TYPE priority_level);
void HblHmi__SetObjectWithPWM(VISUAL_OBJECT_TYPE object_type, uint16 object_index, uint32 object_value, uint8 object_intensity, uint8 pwm_index, HBLHMI_LED_PRIORITY_LEVEL_TYPE priority_level);
void HblHmi__SetVolume(SOUND_VOLUME_TYPE volume_index);
void HblHmi__StopFaultDisplay(void);
void HblHmi__StopAllSound(void);
void HblHmi__SetLedGroupPercentage(uint16 led_group, uint8 percentage, BOOL_TYPE inverse);
void HblHmi__SetLedGroupPercentageWithPWM(uint16 led_group, uint8 percentage, BOOL_TYPE inverse, uint8 pwm_index);
void HblHmi__ModifyLedInGroup(uint16 group_index, uint8 selected, uint8 intensity, HBLHMI_LED_PRIORITY_LEVEL_TYPE priority_level);
void HblHmi__ModifyLedInGroupWithPWM(uint16 group_index, uint8 selected, uint8 intensity, uint8 pwm_index, HBLHMI_LED_PRIORITY_LEVEL_TYPE priority_level);
uint8 HblHmi__GetLedNumInGroup(uint16 led_group);
ON_OFF_TYPE HblHmi__GetLedState(uint16 led_index);
ON_OFF_TYPE HblHmi__GetLedStateInGroup(uint16 group_index, uint8 member_index);
void  HblHmi__InitTableLink (const uint8 *switchLed, uint8 *data, sint8 len, uint8 compartment);

#if (HBLHMI_LED_PRIORITY_FEATURE == ENABLED)
uint32 HblHmi__SetLedPriorityBuffer(HBLHMI_LED_PRIORITY_LEVEL_TYPE priority_level, uint16 led_group, uint32 led_pattern, BOOL_TYPE value);
void HblHmi__ApplyNonAnimationsBuffer(void);
void HblHmi__ApplyAnimationsBuffer(void);
#endif

#endif // HBLHMI_H_
