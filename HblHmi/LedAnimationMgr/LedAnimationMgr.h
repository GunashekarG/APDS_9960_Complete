/**
 *  @file
 *
 *  @brief
 *
 *  $Header: $
 *
 *  @copyright  Copyright 20165-$Date: 2016/04/15 04:55:18EDT $. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
#ifndef LEDANIMATIONMGR_H_
#define LEDANIMATIONMGR_H_

#include "LedAnimationMgr_prm.h"
#include "LedMgr.h"
#include <math.h>

//=====================================================================================================================
//-------------------------------------- PUBLIC (Extern Variables, Constants & Defines) -------------------------------
//=====================================================================================================================

#define SF_LED_BREAK_GROUP 			0xFF
#define LED_ANIMATIONS_PERIODIC		0x1F
#define INVALID_VALUE				0xFF
#define UNLIMITED_DURATION			0x00
#define LED_PATTERN_ANY				0x00
#define ANIMATION_PARAMETERS_SIZE	3

//! Animation types enumerator
typedef enum LED_ANIMATION_TYPE
{
    LED_ANIMATION_FADING	= 0,
    LED_ANIMATION_BLINK     = 1,
    LED_ANIMATION_SEQUENCE  = 2
} LED_ANIMATION_TYPE;

//! Animation status enumerator
typedef enum LED_ANIMATION_STATUS_TYPE
{
    LED_ANIMATION_IDLE		= 0,         			//!< Animation is stopped and memory can be cleared
    LED_ANIMATION_RUNNING	= 1,         			//!< Animation is running
    LED_ANIMATION_EXEC_OVER	= 2,         			//!< Animation current execution is over, need to lead next one
    LED_ANIMATION_PAUSED    = 3,         			//!< Animation is paused
    LED_ANIMATION_WAITING   = 4         			//!< Animation is waiting for repeats delay time to expire
} LED_ANIMATION_STATUS_TYPE;

//! Struct of animation flags
typedef struct LED_ANIMATION_FLAGS_TYPE
{
	uint8 Reverse     			:1;         		//!< Reverse the next animation execution
	uint8 Direction 			:1;         		//!< 0: FadeIn - 1: FadeOut || 0: Led OFF - 1: Led ON || 0: Clockwise - 1: Counterclockwise
	uint8 Clear_On_Execution    :1;         		//!< If set all the animated leds are cleared at the beginning of the animation execution
	uint8 Tail_Scroll_To_End	:1;         		//!< Increases the total steps number to scroll the tail to the end of the animation
	uint8 Inverse_Animation		:1;         		//!< The animation behavior is inverted
	uint8 Cycle_Sequence		:1;         		//!< The animation is performed with a cycling effect (sequences only)
	uint8 Unused				:2;         		//!< unused
}LED_ANIMATION_FLAGS_TYPE;

//! Specific parameter definition for fading effect
PACKED typedef struct
{
	uint8 Current_Intensity; 					    //< Current Intensity %
	uint8 Initial_Intensity; 						//< Initial Intensity %
	uint8 Final_Intensity; 							//< Final Intensity %
}FADING_PARAMETER_TYPE;

//! Specific parameter definition for blink effect
PACKED typedef struct
{
	uint8 Duty_Cycle; 								//< Duty Cycle of the blink animation
}BLINK_PARAMETER_TYPE;

//! Specific parameter definition for sequence effect
PACKED typedef struct
{
	uint8 Step_Num;		    						//< Step Number
	uint8 Steps_Rem;		    					//< Remaining steps counter
	uint8 Tail_Scaling_Num		:3;		    		//< Number of intensity steps to be scaled down for each LEDs tail sub-groupr
	uint8 Tail_Step_Num			:5;		    		//< Number of led sub-groups that will be used as a "tail"
}SEQUENCE_PARAMETER_TYPE;

//! Struct of animation event type
typedef struct LED_ANIMATION_EVENT_TYPE
{
	uint8 Start     			:1; 	        	//!< Generate an event when the animation starts
	uint8 Stop 					:1;     	    	//!< Generate an event when the animation is stopped
	uint8 Pause    				:1;        		 	//!< Generate an event when the animation is paused
	uint8 Resume				:1;        		 	//!< Generate an event when the animation is resumed
	uint8 Step					:1;         		//!< Generate an event when the animation steps
	uint8 Exec_Over				:1;         		//!< Generate an event when the animation has performed a pattern execution
	uint8 Repeat_Over			:1;         		//!< Generate an event when the animation has performed a repeat
	uint8 Expired				:1;         		//!< Generate an event when the animation expires
}LED_ANIMATION_EVENT_TYPE;

//! Defines the led animation parameter structure
PACKED typedef struct LED_ANIMATIONS_TYPE
{
	uint32 Led_Pattern;								//!< Animation Led Pattern
	uint16 Animation_Period;						//!< Animation period (ms)
	uint16 Step_Timer;								//!< Animation step duration (ms)
	uint16 Time_Between_Cycles;						//!< Animation is paused for this duration between two repeats
	uint16 Led_Group;								//!< Animated Led Group
	LED_ANIMATION_TYPE Animation_Type			:3;	//!< 0: Fading - 1: Blink - 2: Sequence
	uint8 Pattern_Executions					:5;	//!< Number of base animation execution
	LED_ANIMATION_FLAGS_TYPE Flags;					//!< Animation flags
	uint8 Repeat_Num 							:5;	//!< Number of animation patterns to be executed within the same repeat
	LED_ANIMATION_STATUS_TYPE Animation_Status	:3; //!< Animation state machine status
	LED_ANIMATIONS_ENUM_TYPE Animation_Index;		//!< Animation Index
	LED_ANIMATION_EVENT_TYPE Event_Mask;			//!< Animation Event Mask
	uint8 Parameters[ANIMATION_PARAMETERS_SIZE];	//!< Animation-specific parameters
	uint8 Pwm_Index;
	BOOL_TYPE Is_Reversing;
}LED_ANIMATIONS_TYPE;

//! Definition of external module events
typedef struct
{
	uint8	LedAnimation_Index;
}LEDANIMATIONMGR_EVENT_TYPE;

typedef void (*LEDANIMATIONMGR_EVENT_HANDLER_TYPE)(LEDANIMATIONMGR_EVENT_TYPE event);

//=====================================================================================================================
//-------------------------------------- PUBLIC (Function Prototypes) -------------------------------------------------
//=====================================================================================================================
void LedAnimationMgr__Initialize(void);
void LedAnimationMgr__Handler(void);

void LedAnimationMgr__Start(LED_ANIMATIONS_ENUM_TYPE animation_index, uint16 led_group, uint32 led_pattern, uint8 num_of_repeats);
void LedAnimationMgr__Pause(uint16 led_group, uint32 led_pattern, uint16 pause_time);
void LedAnimationMgr__Resume(uint16 led_group, uint32 led_pattern);
void LedAnimationMgr__Stop(uint16 led_group, uint32 led_pattern);
void LedAnimationMgr__StopAll(void);

void LedAnimationMgr__RegisterEventHandler(LEDANIMATIONMGR_EVENT_HANDLER_TYPE event_handler);
void LedAnimationMgr__UnregisterEventHandler(LEDANIMATIONMGR_EVENT_HANDLER_TYPE event_handler);

LED_ANIMATIONS_TYPE* LedAnimationMgr__GetAnimationStorePtr(uint8 index);
BOOL_TYPE LedAnimationMgr__IsAnimationRunning(uint8 led_group, uint8* out_anim_index);

#endif // LEDANIMATIONMGR_H_
