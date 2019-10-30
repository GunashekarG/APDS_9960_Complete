/**
 *  @file
 *
 *  @brief      Public interface for the LedMgr module.
 *
 *  $Header: LedMgr.h 1.1 2015/07/23 16:08:18EDT Ryan P Kujawski (KUJAWRP) Exp  $
 *
 *  @copyright  Copyright 2015-$Date: 2015/07/23 16:08:18EDT $. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
#ifndef LEDMGR_H_
#define LEDMGR_H_

//-------------------------------------- Include Files ----------------------------------------------------------------
#include "LedMgr_prm.h"

//=====================================================================================================================
//-------------------------------------- PUBLIC (Extern Variables, Constants & Defines) -------------------------------
//=====================================================================================================================

#define LED_INTENSITY_OFF 0
#define LED_INTENSITY_MAX 100

//! Maximum number of LEDs that can be in a group
#define LED_MAX_GROUP_SIZE 32
#define DIGIT_LED_GROUP_SIZE	7

// List of patterns that can be applied to a group of LEDs, limited by the LED_MAX_GROUP_SIZE
#define PATTERN_NONE (0x00000000UL)
#define PATTERN_ALL (0xFFFFFFFFUL)
#define PATTERN_BIT_SELECTED(select) (0x00000001UL << select)
#define PATTERN_SCALE_TO_SIZE(word) (0x00000000UL | word)

//! This will create a bit pattern in which all bits starting at the start bit and ending at the end bit are set to 1, all others are 0
#define RANGE_TO_PATTERN( start , end ) (((PATTERN_ALL >> start) << start) & (~((PATTERN_ALL >> (end + 1)) << (end + 1))))

//! Value for invalid led group size
#define INVALID_LED_GROUP_SIZE 0xFF

//! Value for invalid led index
#define INVALID_LED_INDEX 0xFF

//! List of possible drivers for LEDs
typedef enum LED_DRIVER_ENUM
{
    LEDMGR_DRIVER_DIRECT = 0,
    LEDMGR_DRIVER_MATRIX,
    LEDMGR_DRIVER_CAT9532,
    LEDMGR_DRIVER_EXTERNAL,
    LEDMGR_DRIVER_PCA9952,
    LEDMGR_DRIVER_ISSI37XX,
    LEDMGR_DRIVER_CUSTOM
}LEDMGR_DRIVER_TYPE;

//! Definition of the configuration for an LED; The parameters are dependent on the driver type
typedef struct
{
    LEDMGR_DRIVER_TYPE Driver_Type;
    uint8              Parameter_1;     // DRIVER_DIRECT = Virtual Pin number, DRIVER_MATRIX = Column,   DRIVER_CAT9532 = Device Index, DRIVER_EXTERNAL = Node ID,   DRIVER_PCA9952 = Device_index
    uint8              Parameter_2;     // DRIVER_DIRECT = Not Used,           DRIVER_MATRIX = Row,      DRIVER_CAT9532 = LED Index,    DRIVER_EXTERNAL = LED Index, DRIVER_PCA9952 = LED Index
    uint8              Parameter_3;     // DRIVER_DIRECT = Not Used,           DRIVER_MATRIX = Not Used, DRIVER_CAT9532 = Not Used,     DRIVER_EXTERNAL = Not Used,  DRIVER_PCA9952 = Not Used
}LEDMGR_CONFIG_TABLE_TYPE;

//=====================================================================================================================
//-------------------------------------- PUBLIC (Function Prototypes) -------------------------------------------------
//=====================================================================================================================
void LedMgr__Initialize(void);
uint16 LedMgr__GetNumOfLeds(void);
uint16 LedMgr__GetNumOfLedsByDriver(LEDMGR_DRIVER_TYPE driver_type);
uint16 LedMgr__GetLedInGroup(uint16 group_index, uint8 member_index);
ON_OFF_TYPE LedMgr__GetLedState(uint16 led_index);
ON_OFF_TYPE LedMgr__GetLedStateInGroup(uint16 group_index, uint8 member_index);
void LedMgr__SetLed(uint16 led_index, uint8 intensity, uint8 pwm_index);
void LedMgr__SetAllLeds(uint8 intensity, uint8 pwm_index);
void LedMgr__SetPatternInGroup(uint16 group_index, uint32 pattern_word, uint8 active_intensity, uint8 pwm_index);
void LedMgr__SetRangeInGroup(uint16 group, uint8 start, uint8 end, uint8 intensity, uint8 pwm_index );
void LedMgr__SetGroup(uint16 group_index, uint8 intensity, uint8 pwm_index);
void LedMgr__SetLedInGroup(uint16 group_index, uint8 member, uint8 intensity, uint8 pwm_index);
void LedMgr__SetGroupFromMemberUp(uint16 group_index, uint8 member, uint8 intensity, uint8 pwm_index);
void LedMgr__SetGroupFromMemberDown(uint16 group_index, uint8 member, uint8 intensity, uint8 pwm_index);
void LedMgr__ModifyLedInGroup(uint16 group_index, uint8 member, uint8 intensity, uint8 pwm_index);
uint8 LedMgr__GetLedNumInGroup(uint16 led_group);
void LedMgr__SetLedState(uint16 led_index);
LEDMGR_CONFIG_TABLE_TYPE* LedMgr__GetLedConfig(uint16 led_index);


#endif // LEDMGR_H_
