/**
 *  @file       
 *
 *  @brief      
 *
 *  $Header: Sequence.h 1.1 2015/08/17 12:28:13EDT Ryan P Kujawski (KUJAWRP) Exp  $
 *
 *  @copyright  Copyright 2015-$Date: 2015/08/17 12:28:13EDT $. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
#ifndef SEQUENCE_H_
#define SEQUENCE_H_

#include "Sequence_prm.h"

//=====================================================================================================================
//-------------------------------------- PUBLIC (Extern Variables, Constants & Defines) -------------------------------
//=====================================================================================================================

//! Definition of data for Sequence
typedef enum
{
    SEQUENCE_START	= 0,
    SEQUENCE_STEP,
    SEQUENCE_COMPLETE,
    SEQUENCE_RESET
} SEQUENCE_PHASE_TYPE;

typedef struct
{
	uint8				Sequence_Event;
}SEQUENCE_EVENT_TYPE;

typedef void (*SEQUENCE_EVENT_HANDLER_TYPE)(SEQUENCE_EVENT_TYPE event);

#define SEQUENCE_EVENT_GET_PHASE(event)   ((event>>6)&(0x03))
#define SEQUENCE_EVENT_GET_INDEX(event)   (event&0x3F)

//=====================================================================================================================
//-------------------------------------- PUBLIC (Function Prototypes) -------------------------------------------------
//=====================================================================================================================
void Sequence__Initialize(void);
void Sequence__Handler(void);
void Sequence__ResetAll(void);

uint32 Sequence__GetSequencesGIIsUsed(uint8 gi_position);

void Sequence__RegisterEventHandler(SEQUENCE_EVENT_HANDLER_TYPE event_handler);
void Sequence__UnregisterEventHandler(SEQUENCE_EVENT_HANDLER_TYPE event_handler);

#endif // SEQUENCE_H_
