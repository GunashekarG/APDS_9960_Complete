/**
 *  @file       
 *
 *  @brief      Controls LEDs that are in a row/column matrix.
 *
 *  @details    This module can process the matrix by row or by column, depending on which is less
 *
 *  $Header: LedMatrix.c 1.1 2015/07/24 16:49:10EDT Ryan P Kujawski (KUJAWRP) Exp  $
 *
 *  @copyright  Copyright 2015-$Date: 2015/07/24 16:49:10EDT $. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------

//-------------------------------------- Include Files ----------------------------------------------------------------
#include "C_Extensions.h"
#include "LedMatrix.h"

#if (LED_DRIVER_MATRIX == ENABLED)
#include "LedMatrix_prv.h"

#include "Gpio.h"
#include "Log.h"
#include <string.h>
#include "Utilities.h"

//-------------------------------------- PRIVATE (Variables, Constants & Defines) -------------------------------------

//! Definition of the output parameters for each column and row
typedef struct
{
    GPIO_PORT_TYPE port;
    uint8 pin;
}LEDMATRIX_PIN_ASSIGNMENT_TYPE;

#ifndef LEDMATRIX_NUM_OF_ROWS
    #error "The number of rows in the LED matrix has to be defined in LedMatrix_prv.h"
#endif

#ifndef LEDMATRIX_NUM_OF_COLS
    #error "The number of columns in the LED matrix has to be defined in LedMatrix_prv.h"
#endif

#ifndef LEDMATRIX_PINS_COLUMNS_DEF
    #error "The LEDMATRIX_PINS_COLUMNS_DEF table needs to be defined in LedMatrix_prv.h"
#else
    static const LEDMATRIX_PIN_ASSIGNMENT_TYPE LEDMATRIX_PINS_COLUMNS[LEDMATRIX_NUM_OF_COLS] = LEDMATRIX_PINS_COLUMNS_DEF;
#endif

#ifndef LEDMATRIX_PINS_ROWS_DEF
    #error "The LEDMATRIX_PINS_ROWS_DEF table needs to be defined in LedMatrix_prv.h"
#else
    static const LEDMATRIX_PIN_ASSIGNMENT_TYPE LEDMATRIX_PINS_ROWS[LEDMATRIX_NUM_OF_ROWS] = LEDMATRIX_PINS_ROWS_DEF;
#endif

//! Default activation of columns is High
#ifndef LEDMATRIX_COLUMN_ACTIVE_HIGH
#define LEDMATRIX_COLUMN_ACTIVE_HIGH		ENABLED
#endif

//! Default activation of rows is High
#ifndef LEDMATRIX_ROW_ACTIVE_HIGH
#define LEDMATRIX_ROW_ACTIVE_HIGH			ENABLED
#endif

//! LEDMATRIX_ACTIVE_HIGH is deprecated
#ifdef LEDMATRIX_ACTIVE_HIGH
#error "LEDMATRIX_ACTIVE_HIGH is deprecated; please remove it and define LEDMATRIX_COLUMN_ACTIVE_HIGH and LEDMATRIX_ROW_ACTIVE_HIGH instead"
#endif

#ifndef LEDMATRIX_FORCE_PRIMARY_ROW
	#define LEDMATRIX_FORCE_PRIMARY_ROW		DISABLED
#endif

#ifndef LEDMATRIX_FORCE_PRIMARY_COLUMN
	#define LEDMATRIX_FORCE_PRIMARY_COLUMN	DISABLED
#endif

#if ((LED_MATRIX_FORCE_PRIMARY_ROW == ENABLED) && (LEDMATRIX_FORCE_PRIMARY_COLUMN == ENABLED))
#error "LED_MATRIX_FORCE_PRIMARY_ROW and LED_MATRIX_FORCE_PRIMARY_COLUMN cannot be both ENABLED"
#endif

//! Define primary and secondary lines based on columns and rows number, to optimize the matrix scanning process - if no explicit request
#if (LEDMATRIX_FORCE_PRIMARY_COLUMN == ENABLED)
	#define LEDMATRIX_PRIMARY_ROW		DISABLED
#else
	#define LEDMATRIX_PRIMARY_ROW		((LEDMATRIX_NUM_OF_COLS > LEDMATRIX_NUM_OF_ROWS) || (LEDMATRIX_FORCE_PRIMARY_ROW == ENABLED))
#endif

#if (LEDMATRIX_PRIMARY_ROW == ENABLED)

#define LEDMATRIX_PRIMARY_LINE_PINS 		LEDMATRIX_PINS_ROWS
#define LEDMATRIX_PRIMARY_LINE_NUM			LEDMATRIX_NUM_OF_ROWS
#define LEDMATRIX_SECONDARY_LINE_PINS 		LEDMATRIX_PINS_COLUMNS
#define LEDMATRIX_SECONDARY_LINE_NUM		LEDMATRIX_NUM_OF_COLS

// Configure primary pins handling
#if (LEDMATRIX_ROW_ACTIVE_HIGH == ENABLED)
#define SET_PRIMARY_PIN_ON(port, pin)		GPIO__PIN_SET((port), (pin))
#define SET_PRIMARY_PIN_OFF(port, pin)		GPIO__PIN_CLR((port), (pin))
#else
#define SET_PRIMARY_PIN_ON(port, pin)		GPIO__PIN_CLR((port), (pin))
#define SET_PRIMARY_PIN_OFF(port, pin)		GPIO__PIN_SET((port), (pin))
#endif

// Configure secondary pins handling
#if (LEDMATRIX_COLUMN_ACTIVE_HIGH == ENABLED)
#define SET_SECONDARY_PIN_ON(port, pin)		GPIO__PIN_SET((port), (pin))
#define SET_SECONDARY_PIN_OFF(port, pin)	GPIO__PIN_CLR((port), (pin))
#else
#define SET_SECONDARY_PIN_ON(port, pin)		GPIO__PIN_CLR((port), (pin))
#define SET_SECONDARY_PIN_OFF(port, pin)	GPIO__PIN_SET((port), (pin))
#endif

#else

#define LEDMATRIX_PRIMARY_LINE_PINS 		LEDMATRIX_PINS_COLUMNS
#define LEDMATRIX_PRIMARY_LINE_NUM 			LEDMATRIX_NUM_OF_COLS
#define LEDMATRIX_SECONDARY_LINE_PINS 		LEDMATRIX_PINS_ROWS
#define LEDMATRIX_SECONDARY_LINE_NUM		LEDMATRIX_NUM_OF_ROWS

// Configure primary pins handling
#if (LEDMATRIX_COLUMN_ACTIVE_HIGH == ENABLED)
#define SET_PRIMARY_PIN_ON(port, pin)		GPIO__PIN_SET((port), (pin))
#define SET_PRIMARY_PIN_OFF(port, pin)		GPIO__PIN_CLR((port), (pin))
#else
#define SET_PRIMARY_PIN_ON(port, pin)		GPIO__PIN_CLR((port), (pin))
#define SET_PRIMARY_PIN_OFF(port, pin)		GPIO__PIN_SET((port), (pin))
#endif

// Configure secondary pins handling
#if (LEDMATRIX_ROW_ACTIVE_HIGH == ENABLED)
#define SET_SECONDARY_PIN_ON(port, pin)		GPIO__PIN_SET((port), (pin))
#define SET_SECONDARY_PIN_OFF(port, pin)	GPIO__PIN_CLR((port), (pin))
#else
#define SET_SECONDARY_PIN_ON(port, pin)		GPIO__PIN_CLR((port), (pin))
#define SET_SECONDARY_PIN_OFF(port, pin)	GPIO__PIN_SET((port), (pin))
#endif

#endif

//! The name of this module for use by the Log module.
#define MODULE_NAME MODULE_LEDMATRIX

//! A list of log message IDs that are generated by this module.
typedef enum MODULE_LEDMATRIX_LOG_MESSAGE_ID_ENUM
{
	LED_COLUMN_OUT_OF_RANGE = 1,
	LED_ROW_OUT_OF_RANGE = 2
} MODULE_LEDMATRIX_LOG_MESSAGE_ID_TYPE;

//!< Keeps track of the logged messages sent, to prevent log spam
static uint8 LedMatrix_Log_Counter;

#ifndef LEDMATRIX_LOG_LIMIT
	#define LEDMATRIX_LOG_LIMIT	10
#endif

//	//!< Keeps track of the current row or column that is being processed
static uint8 LedMatrix_Scan_Index;

//! Array of desired states for the LEDs. Each entry in the array corresponds to a row in the matrix and each bit in an entry
//! corresponds to a column in the matrix.
#if (LEDMATRIX_NUM_OF_COLS <= 8)
    static uint8 LedMatrix_Output_States[LEDMATRIX_NUM_OF_ROWS];
    #define LEDMATRIX_COLUMN_DATA_SIZE 8
#elif (LEDMATRIX_NUM_OF_COLS <= 16)
    static uint16 LedMatrix_Output_States[LEDMATRIX_NUM_OF_ROWS];
    #define LEDMATRIX_COLUMN_DATA_SIZE 16
#else
    #error "LED matrix with more than 16 columns is not supported"
#endif

//-------------------------------------- PRIVATE (Function Prototypes) ------------------------------------------------
static void MatrixOutputConfig(void);

//=====================================================================================================================
//-------------------------------------- Public Functions -------------------------------------------------------------
//=====================================================================================================================

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief   Initializes the module variables
 */
void LedMatrix__Initialize(void)
{
    memset(&LedMatrix_Output_States, 0, sizeof(LedMatrix_Output_States));
    LedMatrix_Scan_Index = LEDMATRIX_PRIMARY_LINE_NUM;
    LedMatrix_Log_Counter = 0;
    MatrixOutputConfig();
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief   Tasks to be executed every 1ms.
 * @details Handles the column-row processing of the matrix
 */
void LedMatrix__Handler1ms(void)
{
    uint8 index;
    #if (LEDMATRIX_NUM_OF_COLS <= 8)
        uint8 temp_outputs_bytes[LEDMATRIX_NUM_OF_ROWS];
    #elif (LEDMATRIX_NUM_OF_COLS <= 16)
        uint16 temp_outputs_bytes[LEDMATRIX_NUM_OF_ROWS];
    #endif

	// Turn OFF all the primary lines
	for (index = 0; index < LEDMATRIX_PRIMARY_LINE_NUM; index++)
	{
		SET_PRIMARY_PIN_OFF(LEDMATRIX_PRIMARY_LINE_PINS[index].port, LEDMATRIX_PRIMARY_LINE_PINS[index].pin);       //lint !e929 cast from pointer to pointer [MISRA 2004 Rule 11.4]
	}

	// -- Prepare new scan index -- //
	LedMatrix_Scan_Index++;
	if (LedMatrix_Scan_Index >= LEDMATRIX_PRIMARY_LINE_NUM)
	{
		LedMatrix_Scan_Index = 0;
	}

	// Copy desired LED states into temporary buffer
	memcpy(&temp_outputs_bytes[0], &LedMatrix_Output_States[0], sizeof(temp_outputs_bytes));
	// Turn the appropriate secondary line on or off based on the current primary line
	for (index = 0; index < LEDMATRIX_SECONDARY_LINE_NUM; index++)
	{
#if (LEDMATRIX_PRIMARY_ROW == ENABLED)
		if (BIT_TEST(temp_outputs_bytes[LedMatrix_Scan_Index], index))
#else
		if (BIT_TEST(temp_outputs_bytes[index], LedMatrix_Scan_Index))
#endif
		{
			// Set "on" secondary lines
			SET_SECONDARY_PIN_ON(LEDMATRIX_SECONDARY_LINE_PINS[index].port, LEDMATRIX_SECONDARY_LINE_PINS[index].pin);     //lint !e929 cast from pointer to pointer [MISRA 2004 Rule 11.4]
		}
		else
		{
			SET_SECONDARY_PIN_OFF(LEDMATRIX_SECONDARY_LINE_PINS[index].port, LEDMATRIX_SECONDARY_LINE_PINS[index].pin);     //lint !e929 cast from pointer to pointer [MISRA 2004 Rule 11.4]
		}
	}

	// Enable current primary line
	SET_PRIMARY_PIN_ON(LEDMATRIX_PRIMARY_LINE_PINS[LedMatrix_Scan_Index].port, LEDMATRIX_PRIMARY_LINE_PINS[LedMatrix_Scan_Index].pin);     //lint !e929 cast from pointer to pointer [MISRA 2004 Rule 11.4]

}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief   Sets the state of a single LED.
 * @details If LED at led_index is not part of the LED matrix, it should be configured with a column and row value of 0xF
 *          in the LEDMATRIX_CR_TABLE.
 * @param   led_index: The index number corresponding to the LED being manipulated.
 * @param   led_state: The desired state of the LED. TRUE if the LED should be on and FALSE if the LED should be off.
 */
void LedMatrix__SetLed(uint8 column, uint8 row, BOOL_TYPE led_state)
{
    uint8 output_bit;

    if(column < LEDMATRIX_NUM_OF_COLS &&
    	row < LEDMATRIX_NUM_OF_ROWS)
    {
        // Determine the bit to set in the column-row array
        output_bit = (row * LEDMATRIX_COLUMN_DATA_SIZE) + column;

        // Set the output bit in the column-row array
        if (output_bit < (sizeof(LedMatrix_Output_States) * 8))
        {
            Utilities__SetBitInArray((uint8 *)LedMatrix_Output_States, output_bit, led_state);      //lint !e928 cast from pointer to pointer [MISRA 2004 Rule 11.4]
        }
    }
    else if(LedMatrix_Log_Counter < LEDMATRIX_LOG_LIMIT)
    {
        LedMatrix_Log_Counter++;

        if(column >= LEDMATRIX_NUM_OF_COLS)
        {
            LOG_ADD_EXCEPTION(LED_COLUMN_OUT_OF_RANGE, (LOG_DATA_TYPE)column);
        }
        if(row >= LEDMATRIX_NUM_OF_ROWS)
        {
			LOG_ADD_EXCEPTION(LED_ROW_OUT_OF_RANGE, (LOG_DATA_TYPE)row);
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief		Returns current primary line index
 * @return    Current primary line index
 */
uint8 LedMatrix__GetCurrentLine(void)
{
	return(LedMatrix_Scan_Index);
}

//=====================================================================================================================
//-------------------------------------- Private Functions ------------------------------------------------------------
//=====================================================================================================================

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief  Configures the outputs for the rows and columns.
 */
static void MatrixOutputConfig(void)
{
    uint8 index;

    // configure columns
    for (index = 0; index < LEDMATRIX_NUM_OF_COLS; index++)
    {
        GPIO__PIN_COFIG_O_PUSHPULL(LEDMATRIX_PINS_COLUMNS[index].port, LEDMATRIX_PINS_COLUMNS[index].pin);
    }

    // configure rows
    for (index = 0; index < LEDMATRIX_NUM_OF_ROWS; index++)
    {
        GPIO__PIN_COFIG_O_OPENDRAIN(LEDMATRIX_PINS_ROWS[index].port, LEDMATRIX_PINS_ROWS[index].pin);
    }
}

#endif // (LED_DRIVER_MATRIX == ENABLED)
