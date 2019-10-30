/**
 *  @file       
 *
 *  @brief      
 *
 *  $Header: LedMatrix.h 1.1 2015/07/24 16:49:13EDT Ryan P Kujawski (KUJAWRP) Exp  $
 *
 *  @copyright  Copyright 2015-$Date: 2015/07/24 16:49:13EDT $. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
#ifndef LEDMATRIX_H_
#define LEDMATRIX_H_

#include "SystemConfig.h"

#if (LED_DRIVER_MATRIX == ENABLED)

//=====================================================================================================================
//-------------------------------------- PUBLIC (Extern Variables, Constants & Defines) -------------------------------
//=====================================================================================================================


//=====================================================================================================================
//-------------------------------------- PUBLIC (Function Prototypes) -------------------------------------------------
//=====================================================================================================================
void LedMatrix__Initialize(void);
void LedMatrix__Handler1ms(void);
uint8 LedMatrix__GetCurrentLine(void);
void LedMatrix__SetLed(uint8 column, uint8 row, BOOL_TYPE led_state);

#endif // (LED_DRIVER_MATRIX == ENABLED)
#endif // LEDMATRIX_H_
