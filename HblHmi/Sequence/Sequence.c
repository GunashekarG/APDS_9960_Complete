/**
 *  @file       
 *
 *  @brief      Handles key sequences.
 *
 *  @details    - Executes a sequence based on key events received from the HBL layer.
 *              - A sequence is cancelled if interrupted by a key press not in the sequence.
 *              - A sequence is cancelled if a key is released on a step before the long press timeout.
 *              - Only one sequence can be running at a time
 *
    @startuml{Sequence_Class_Diagram.png}
    title Sequence Class Diagram

    hide empty attributes
    hide empty methods
    hide circle

    class "Sequence" as core {
        +Sequence__Initialize(void) : void
        +Sequence__Handler(void) : void
        +Sequence__ResetAll(void) : void
		+Sequence__GetSequencesGIIsUsed(uint8 gi_position) : uint32
		+Sequence__RegisterEventHandler(SEQUENCE_EVENT_HANDLER_TYPE event_handler) : void
		+Sequence__UnregisterEventHandler(SEQUENCE_EVENT_HANDLER_TYPE event_handler) : void
        -AreStepConditionsMet(uint8 seq_index, HBLHMI_GI_EVENT_TYPE event) : BOOL_TYPE
        -GenericInputEventHandler(HBLHMI_GI_EVENT_TYPE event) : void
        -LoadStep(uint8 index) : PASS_FAIL_TYPE
        -ResetSequence(uint8 index) : void
        -SetTargetGI(uint8 index) : void
        -ExecuteCallBack(uint8 sequence_index, SEQUENCE_PHASE_TYPE sequence_phase) : void
        -GetGIsUsedInSequences(void) : void
    }

    core ..|> HblHmi
    core ..|> Log
    core ..|> SystemTimers

    @enduml

    @startuml{Sequence_Usecase_1.png}
    title Sequence Sequence Diagram Valid sequence is:\n1) Digital GI 1 duration: 0 timeout: x > 0\n2) Digital GI 2 duration: 0 timeout: y > 0
    hide empty attributes
    hide empty methods
    hide circle

    actor User
	entity Sequence

    alt Correct sequence entered

    else Incorrect sequence
    	alt Wrong second GI

    	else First step too long

    	else No second step within timeout

    end

    @enduml

    @startuml{Sequence_Usecase_2.png}
    title Sequence Sequence Diagram Valid sequence is:\n1) Digital GI 1 duration: x > 0 timeout: y > x \n2) Digital GI 2 duration: z > 0 timeout: p > z
    hide empty attributes
    hide empty methods
    hide circle

    actor User
	entity Sequence

    alt Correct sequence entered

    else Incorrect sequence
    	alt First step too short

    	else First step too long

    	else second step too short

    	else second step too long

    end

    @enduml

    @startuml{Sequence_Usecase_3.png}
    title Sequence Sequence Diagram Valid sequence is:\n1) Analog GI 1 value = 0, duration: 0, timeout: x > 0\n2) Analog GI 1 value = 0, duration: 0 ,timeout: y > 0
    hide empty attributes
    hide empty methods
    hide circle

    actor User
	entity Sequence

    alt Correct sequence entered

    else Incorrect sequence
    	alt Wrong second GI

    	else First step too long

    	else No second step within timeout

    end

    @enduml

    @startuml{Sequence_Usecase_4.png}
    title Sequence Sequence Diagram Valid sequence is:\n1) Analog GI 1 value = 0 ,duration: x > 0, timeout: y > x \n2) Analog GI 1 value = 0,duration: z > 0, timeout: p > z
    hide empty attributes
    hide empty methods
    hide circle

    actor User
	entity Sequence

    alt Correct sequence entered

    else Incorrect sequence
    	alt First step too short

    	else First step too long

    	else second step too short

    	else second step too long

    end

    @enduml
 *
 *  $Header: Sequence.c 1.2 2015/08/21 11:39:52EDT Ryan P Kujawski (KUJAWRP) Exp  $
 *
 *  @copyright  Copyright 2015-$Date: 2015/08/21 11:39:52EDT $. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------

//-------------------------------------- Include Files ----------------------------------------------------------------
#include "SystemConfig.h"
#include "Sequence.h"
#include "Sequence_prv.h"

#include "Callback.h"
#include "HblHmi.h"
#include "Log.h"
#include "RegulationFilter.h"
#include <string.h>
#include "SystemTimers.h"
#include "Mode.h"

//-------------------------------------- PRIVATE (Variables, Constants & Defines) -------------------------------------

//! The name of this module for use by the Log module.
#define MODULE_NAME MODULE_SEQUENCE

//! A list of log message IDs that are generated by this module.
typedef enum MODULE_SEQUENCE_LOG_MESSAGE_ID_ENUM
{
    MAX_NUM_SEQUENCES_EXCEEDED      = 1,     // Data: Number of sequences in the Setting File
    MAX_NUM_GI_CONDITIONS_EXCEEDED  = 2,     // Data: The sequence that exceeded the max number of GIs (the Num_Of_Step_Conditions in SettingFile is bigger than MAX_NUM_HANDLED_GI_IN_STEP_CONDITIONS)
    FAILED_TO_GET_SYSTEM_TIMER      = 3,
	MAX_NUM_OF_STEPS_EXCEEDED		= 4		 // Data: The sequence that exceeded the max number of steps
} MODULE_SEQUENCE_LOG_MESSAGE_ID_TYPE;

//! Sequence step header type
PACKED typedef struct SEQ_STEP_DATA_STRUCT
{
    uint8 Timeout        :4;    // Timeout for a single step in a sequence (in seconds)
    uint8 Timer          :4;    // Timeout for a long key press for a single step in a sequence (in seconds)
    uint8 Feedback_Code  :4;
    uint8 Unused         :3;
    uint8 Is_Function_Layer_Disabled :1;
}SEQ_STEP_DATA_TYPE;

//! Condition GI union
PACKED typedef union GI_ADDR_UNION
{
    uint8 Byte;
    struct
    {
        uint8               Position    :6;
        HBLHMI_GI_READ_TYPE Read_Type   :2;
    }Bits;
}GI_ADDR_TYPE;

//! Sequence header struct
PACKED typedef struct SEQUENCE_DATA_STRUCT
{
    GI_ADDR_TYPE GI_Address;
    uint8 GI_Seq_Value;
    uint8 Num_Of_Steps      :5;
    uint8 Unused            :2;
    uint8 Write_To_GI_New   :1;
}SEQUENCE_DATA_TYPE;

//! Sequence struct
PACKED typedef struct SEQUENCE_STRUCT
{
    GI_ADDR_TYPE Target_GI_Address;
    GI_ADDR_TYPE * Condition_GI_Address;
    uint8 Target_GI_Value;
    uint8 Current_Step;
    uint8 Remaining_Time;   // In seconds
    uint8 Num_Of_Steps;
    uint8 Num_Of_Step_Conditions;
    uint16 Step_Timeout_Handle;          //!< Handle used to keep track of the variable used by the SystemTimers module
    uint16 Key_Press_Timeout_Handle;     //!< Handle used to keep track of the variable used by the SystemTimers module
    SEQ_STEP_DATA_TYPE * Step_Data_Ptr;
}SEQUENCE_TYPE;

//! Defines the maximum number of sequences the module will support
//! The Maximum number of sequences the system can handle is 32
//! Above 32 the mechanism (HmiEvenMgr) that prevents Regulations changes while executing sequences will not work
#ifndef MAX_NUM_SEQUENCES
    //! Keep the maximum at 10 as default so we don't eat too many RAM for nothing.
    #define MAX_NUM_SEQUENCES 10
#endif
//! Ensure the number of Sequences will be up to 32
CT_ASSERT(MAX_NUM_SEQUENCES <= 32);

static SEQUENCE_TYPE Sequence[MAX_NUM_SEQUENCES];    //!< Data for each sequence
static uint8 Num_Of_Sequences;                       //!< Number of defined sequences in the Setting File

#ifndef SEQUENCE_CALLBACK_REGISTER_SIZE
    #define SEQUENCE_CALLBACK_REGISTER_SIZE 1
#endif

DEFINE_CALLBACK_CONTROL_STRUCTURE(Sequence_Callback_Control_Structure, SEQUENCE_CALLBACK_REGISTER_SIZE);

//! This number is defined based on the maximum "Num_Of_Step_Conditions" is expected for a sequence configuration coming from GESE
//! for example: If it is expected that a sequence in GESE will involve 3 different GIs this number should be at least 3
#ifndef MAX_NUM_HANDLED_GI_IN_STEP_CONDITIONS
    #define MAX_NUM_HANDLED_GI_IN_STEP_CONDITIONS (3)
#endif

//! This number defines max number of steps in the sequence.
#ifndef MAX_NUM_STEPS_IN_SEQUENCE
	#define MAX_NUM_STEPS_IN_SEQUENCE (5)
#endif

//! Ensure the number of Sequence steps will be up to 31
CT_ASSERT(MAX_NUM_STEPS_IN_SEQUENCE <= 31);

//! Structure to hold information of Sequence Step Conditions
typedef struct SEQUENCE_CONDITION_STRUCT
{
    uint8 Used_GI[MAX_NUM_STEPS_IN_SEQUENCE][MAX_NUM_HANDLED_GI_IN_STEP_CONDITIONS];   //!< GIs used in the step conditions
}SEQUENCE_CONDITIONS_TYPE;

//! Variable holding information of Sequence Step Conditions
static SEQUENCE_CONDITIONS_TYPE Sequence_Conditions[MAX_NUM_SEQUENCES];

//-------------------------------------- PRIVATE (Function Prototypes) ------------------------------------------------
static BOOL_TYPE AreStepConditionsMet(uint8 seq_index, HBLHMI_GI_EVENT_TYPE event);
static void GenericInputEventHandler(HBLHMI_GI_EVENT_TYPE event);
static PASS_FAIL_TYPE LoadStep(uint8 index);
static void ResetSequence(uint8 index);
static void SetTargetGI(uint8 index);
static void ExecuteCallBack(uint8 sequence_index, SEQUENCE_PHASE_TYPE sequence_phase);
static void GetGIsUsedInSequences(void);
//=====================================================================================================================
//-------------------------------------- Public Functions -------------------------------------------------------------
//=====================================================================================================================

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief  Initializes the module Sequence and its variables
 */
void Sequence__Initialize(void)
{
    HblHmi__RegisterSequenceHandler(&GenericInputEventHandler);
    Num_Of_Sequences = SettingFile__GetNumDisplacements(0, SF_PTR_UI_SEQUENCER_DATA);
    Callback__Initialize(&Sequence_Callback_Control_Structure);

    // Log message if the Setting File has defined more sequences than the module can handle
    if (Num_Of_Sequences > MAX_NUM_SEQUENCES)
    {
        LOG_ADD_ERROR(MAX_NUM_SEQUENCES_EXCEEDED, Num_Of_Sequences);

        //putting the system in ERROR since if the maximum number gets exceed the system resets
        Mode__SetModeExtended(MODE_NORMAL, SUBMODE_NORMAL_ERROR);
        //none Sequence shall be allowed to run, due to settingFile read returned more sequences than the module can handle.
        Num_Of_Sequences = 0;
    }
    else
    {
        // Loop through all the sequences and allocate a system time in case the sequence has a step with a long press
        for (uint8 index = 0; index < Num_Of_Sequences; index++)
        {
            Sequence[index].Step_Timeout_Handle = SystemTimers__GetHandle(SYSTEMTIMERS_TYPE_DOWN_COUNTER);
            Sequence[index].Key_Press_Timeout_Handle = SystemTimers__GetHandle(SYSTEMTIMERS_TYPE_DOWN_COUNTER);

            if ((Sequence[index].Step_Timeout_Handle == SYSTEMTIMERS_FAIL) ||
                (Sequence[index].Key_Press_Timeout_Handle == SYSTEMTIMERS_FAIL))
            {
                LOG_ADD_EXCEPTION(FAILED_TO_GET_SYSTEM_TIMER, index);
                break;  // No need to keep reporting this exception if even one timer cannot be allocated for this module
            }
        }

        memset(Sequence_Conditions, HBLHMI_GI_INVALID, sizeof(Sequence_Conditions));
        GetGIsUsedInSequences();
        Sequence__ResetAll();   //Sequence variable was used to get the GIs, clean it.
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief   Background tasks to manage the sequences.
 */
void Sequence__Handler(void)
{
    uint8 index;

    for (index = 0; index < Num_Of_Sequences; index++) 											// For each sequence
    {
        if (SystemTimers__GetEvent(Sequence[index].Step_Timeout_Handle) == TRUE)				// If timeout for the current step occurred.
        {
            ResetSequence(index);																// Reset the sequence and
            ExecuteCallBack(index, SEQUENCE_RESET);												// call callback to inform about sequence reset
        }
        else if (SystemTimers__GetEvent(Sequence[index].Key_Press_Timeout_Handle) == TRUE)      // if the required step time elapsed
        {
            if ((++Sequence[index].Current_Step) >= Sequence[index].Num_Of_Steps)				// increment the step and check if number of steps reached
            {
                SetTargetGI(index);																// Set target GI and
                ExecuteCallBack(index, SEQUENCE_COMPLETE);										// call callback to inform about sequence complete
            }
            else    // Load next step
            {
                ExecuteCallBack(index, SEQUENCE_STEP);											// there are still steps to be executed but this is completed
            }
            STEP_COMPLETED(Sequence[index].Step_Data_Ptr->Feedback_Code);						// Step completed feedback
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief   Resets all the sequences.
 *
 *  @details This function can be used to interrupt and reset sequences based on external behaviors.
 */
void Sequence__ResetAll(void)
{
    uint8 index;

    for (index = 0; index < Num_Of_Sequences; index++)
    {
        ResetSequence(index);
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief   Registers an event handler for Sequences with the module.
 * @param   event_handler: sequence data
 */
void Sequence__RegisterEventHandler(SEQUENCE_EVENT_HANDLER_TYPE event_handler)
{
    Callback__Register(&Sequence_Callback_Control_Structure, (CALLBACK_HANDLER_TYPE)event_handler);  //lint !e929 Suppress Info: cast from pointer to pointer [MISRA 2004 Rule 11.4]
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief   Unregisters an event handler for Sequences with the module.
 * @param   event_handler: data
 */
void Sequence__UnregisterEventHandler(SEQUENCE_EVENT_HANDLER_TYPE event_handler)
{
    Callback__Unregister(&Sequence_Callback_Control_Structure, (CALLBACK_HANDLER_TYPE)event_handler);    //lint !e929 Suppress Info: cast from pointer to pointer [MISRA 2004 Rule 11.4]
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief   It sets a bit for each sequence using the gi_position
 * @details Currently function supports only digital GI.
 * @param   gi_position is the GI under test of what sequences are using it
 * @return  A bitwise value for each sequence using the GI under test in current step
 */
uint32 Sequence__GetSequencesGIIsUsed(uint8 gi_position)
{
    uint8 seq_index, step_index, cond_index;
    uint32 sequences_using_this_gi = 0;
    BOOL_TYPE found;

    for (seq_index = 0; seq_index < Num_Of_Sequences; seq_index++)
    {
    	found = FALSE;
    	for(step_index = 0; step_index < Sequence[seq_index].Num_Of_Steps; step_index++)
        {
    		for(cond_index = 0; cond_index < MAX_NUM_HANDLED_GI_IN_STEP_CONDITIONS; cond_index++)
    		{
    			if(Sequence_Conditions[seq_index].Used_GI[step_index][cond_index] == gi_position)
    			{
    				BIT_SET(sequences_using_this_gi, seq_index);
    				found = TRUE;
    				break;	// Jump out of conditions loop
    			}
    		}
    		if(found == TRUE)
    		{
    			break; // Jump to next sequence
    		}
        }
    }

    return(sequences_using_this_gi);
}

//=====================================================================================================================
//-------------------------------------- Private Functions ------------------------------------------------------------
//=====================================================================================================================

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief   Handles key events to manage the sequences.
 */
static void GenericInputEventHandler(HBLHMI_GI_EVENT_TYPE event)
{
    SYSTEMTIMER_HMS_TYPE hms_timer;
    uint8 index;

    void* gi_data = HblHmi__GetGIData(event.GI_Read_Type, event.GI_Position);		//Get the data of changed GI

    for (index = 0; index < Num_Of_Sequences; index++)								// For each sequence
    {
    	if (!(((event.GI_Read_Type == HBLHMI_GI_READ_DIGITAL) && ((KEYGI_DATA_TYPE*)gi_data)->Current_State == HBL_KEY_RELEASED)) && // Ignore digital GI releases
    		(AreStepConditionsMet(index, event))) 								    // Load data for this sequence from the Setting File
    	{
    		if(Sequence[index].Current_Step == 0)       							// If it's the first step
    		{
    			ExecuteCallBack(index, SEQUENCE_START);
    		}

    		if (SystemTimers__GetState(Sequence[index].Key_Press_Timeout_Handle) == SYSTEMTIMERS_STATE_RUNNING) // Check if step with long key press was interrupted by another key press
    		{
    			ResetSequence(index);
    			LONG_PRESS_STEP_INTERRUPTED();
    		}
    		else if(Sequence[index].Remaining_Time != 0)							// If timed step
			{
				memset(&hms_timer, 0, sizeof(hms_timer));
				hms_timer.seconds = Sequence[index].Remaining_Time;
				SystemTimers__SetHMS(Sequence[index].Key_Press_Timeout_Handle, &hms_timer);	// Set timed event timer
			}
    		else																	// Not timed event
    		{
    			Sequence[index].Current_Step += 1;									// Increment step
    			STEP_COMPLETED(Sequence[index].Step_Data_Ptr->Feedback_Code);
    			if (Sequence[index].Current_Step >= Sequence[index].Num_Of_Steps)   //Check if this is the end of the sequence
    			{
    				SetTargetGI(index);
    				ExecuteCallBack(index, SEQUENCE_COMPLETE);
    			}
    		}
    	} // AreStepConditionsMet()
    	else
    	{
    		if(!((event.GI_Read_Type == HBLHMI_GI_READ_DIGITAL) &&
    		    ((KEYGI_DATA_TYPE*)gi_data)->Current_State == HBL_KEY_PRESSED)) // TRUE for all non-digital GIs and released digital GIs.
    		{
				for(uint8 condition = 0, condition_ptr = 0; condition < Sequence[index].Num_Of_Step_Conditions; condition++, condition_ptr++) // For each condition
				{
					if ((event.GI_Position == Sequence[index].Condition_GI_Address[condition_ptr].Bits.Position) &&				 // Find if GI is one of condition GIs
						(event.GI_Read_Type == Sequence[index].Condition_GI_Address[condition_ptr].Bits.Read_Type) &&
						(Sequence[index].Remaining_Time != 0) &&                       											 // Only process steps that use a long key press
						(SystemTimers__GetState(Sequence[index].Key_Press_Timeout_Handle) == SYSTEMTIMERS_STATE_RUNNING))        // If timer not expired, sequence ended prematurely
					{
						if((event.GI_Read_Type == HBLHMI_GI_READ_DIGITAL) ||													 // If condition digital GI released before event time expiration
						   ((((Sequence[index].Condition_GI_Address[0].Bits.Read_Type == HBLHMI_GI_READ_ENCODER) &&				 // OR GI is encoder and value changed prematurely
							 ((uint8)(((ENCGI_DATA_TYPE *)gi_data)->Current_Enc_Value) != Sequence[index].Condition_GI_Address[condition_ptr + 1].Byte)) ||
						   ((Sequence[index].Condition_GI_Address[0].Bits.Read_Type == HBLHMI_GI_READ_ANALOG) &&				// OR GI is analog and value changed prematurely
							((uint8)(((ANALOGGI_DATA_TYPE *)gi_data)->Current_Value) != Sequence[index].Condition_GI_Address[condition_ptr + 1].Byte)))))
						{
							ResetSequence(index);
							LONG_PRESS_STEP_INCOMPLETE();
							ExecuteCallBack(index, SEQUENCE_RESET);
						}
					}

					if(Sequence[index].Condition_GI_Address[condition_ptr].Bits.Read_Type != HBLHMI_GI_READ_DIGITAL)
					{
						condition_ptr++;
					}
				}
    		}
    	}
    } //end for loop
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief   Determines if the step is complete based on the key that was pressed or if all keys for the step are pressed.
 * @param   seq_index
 * @param   event
 * @return
 */
static BOOL_TYPE AreStepConditionsMet(uint8 seq_index, HBLHMI_GI_EVENT_TYPE event)
{
    BOOL_TYPE is_step_complete;
    void* gi_data;

    is_step_complete = FALSE;

    if (LoadStep(seq_index) == PASS)							// Step correctly loaded
    {
        if (Sequence[seq_index].Num_Of_Step_Conditions > 1)		// More than one condition in the step
        {
            uint8 gi_in_current_sequence_step = FALSE;

            // Go through all GIs for the current step of the sequence and look for a match
            for (uint8 index = 0, condition_ptr = 0; index < Sequence[seq_index].Num_Of_Step_Conditions; index++, condition_ptr++)
            {
                if ((event.GI_Position == Sequence[seq_index].Condition_GI_Address[condition_ptr].Bits.Position) &&
                    (event.GI_Read_Type == Sequence[seq_index].Condition_GI_Address[condition_ptr].Bits.Read_Type))
                {
                    gi_in_current_sequence_step = TRUE;
                    break;
                }

                if(Sequence[seq_index].Condition_GI_Address[condition_ptr].Bits.Read_Type != HBLHMI_GI_READ_DIGITAL)
                {
                	condition_ptr++;
                }
            }

            // If the GI event matched at least one GI in the current step of the sequence, allow the sequence to continue
            // Next check is if all the conditions of the step are satisfied
            if (gi_in_current_sequence_step == TRUE)
            {
                is_step_complete = TRUE;
                uint8 expected_value = 0;
                uint8 current_value = 0xFF;
                GI_ADDR_TYPE* condition_ptr = &(Sequence[seq_index].Condition_GI_Address[0]);					// Set GI condition pointer on first condition

                // Verify that all condition GIs are in expected state
                for (uint8 index = 0; index < Sequence[seq_index].Num_Of_Step_Conditions; index++)
                {
                	gi_data = HblHmi__GetGIData(condition_ptr->Bits.Read_Type, condition_ptr->Bits.Position);	// Get condition GI data

                	/* Depending on the GI type, get current and expected value*/
                	switch(condition_ptr->Bits.Read_Type)
                	{
                	case HBLHMI_GI_READ_DIGITAL:
                		expected_value = (uint8)HBL_KEY_PRESSED;
                		current_value = (uint8)(((KEYGI_DATA_TYPE *)gi_data)->Current_State);
                		break;

					case HBLHMI_GI_READ_ENCODER:
                		condition_ptr++;
						expected_value = (uint8)(*((uint8*)condition_ptr)); 						//lint !e928 cast from pointer to pointer [MISRA 2004 Rule 11.4]
                		current_value = (uint8)(((ENCGI_DATA_TYPE *)gi_data)->Current_Enc_Value);
                		break;

					case HBLHMI_GI_READ_ANALOG:
                		condition_ptr++;
						expected_value = (uint8)(*((uint8*)condition_ptr));							//lint !e928 cast from pointer to pointer [MISRA 2004 Rule 11.4]
                		current_value = (uint8)(((ANALOGGI_DATA_TYPE *)gi_data)->Current_Value);
                		break;

					default:

						break;
                	}

                	if(expected_value != current_value)		// Verify the current value is the one expected.
                	{
                		is_step_complete = FALSE;			// Step is not completed if any of condition GIs is in state different than expected
                		break;
                	}
                	condition_ptr++;
                }
            }
            else    // GI did not match any of the GIs in the current step of the sequence
            {
                ResetSequence(seq_index);
            }
        }
        else 		// Only one condition in the step
        {
            if ((event.GI_Position == Sequence[seq_index].Condition_GI_Address[0].Bits.Position) &&			// GI is the one
                (event.GI_Read_Type == Sequence[seq_index].Condition_GI_Address[0].Bits.Read_Type))			// step expects
            {
            	gi_data = HblHmi__GetGIData(Sequence[seq_index].Condition_GI_Address[0].Bits.Read_Type,
            								Sequence[seq_index].Condition_GI_Address[0].Bits.Position);	    // Get the data of the GI

            	if((Sequence[seq_index].Condition_GI_Address[0].Bits.Read_Type == HBLHMI_GI_READ_DIGITAL) && // If digital,
            	   (((KEYGI_DATA_TYPE *)gi_data)->Current_State == HBL_KEY_PRESSED))						 // and key pressed
            	{
            		is_step_complete = TRUE;																// condition met
            	}
            	else																						// If NOT digital
            	{
            		/* Verify that GI data is the one expected by the step condition */
            		if(((Sequence[seq_index].Condition_GI_Address[0].Bits.Read_Type == HBLHMI_GI_READ_ENCODER) &&
            		    ((uint8)(((ENCGI_DATA_TYPE *)gi_data)->Current_Enc_Value) == Sequence[seq_index].Condition_GI_Address[1].Byte)) ||
            		   ((Sequence[seq_index].Condition_GI_Address[0].Bits.Read_Type == HBLHMI_GI_READ_ANALOG) &&
            		    ((uint8)(((ANALOGGI_DATA_TYPE *)gi_data)->Current_Value) == Sequence[seq_index].Condition_GI_Address[1].Byte)))
            		{
            			is_step_complete = TRUE;
            		}
            		else
            		{
            			ResetSequence(seq_index);       // GI is OK but the value is NOT - reset the sequence.
            		}
            	}
            }
            else
            {
                ResetSequence(seq_index);       // Sequence should not start/continue if a key is pressed out of sequence
            }
        }
    }
    // else, failed to load step

    return(is_step_complete);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief   Loads the data from the SettingFile for a sequence.
 */
static PASS_FAIL_TYPE LoadStep(uint8 index)
{
    SYSTEMTIMER_HMS_TYPE hms_timer;
    SEQUENCE_DATA_TYPE * sequence_data_ptr;
    SETTINGFILE_LOADER_TYPE sf_loader;
    uint8 * step_ptr;
    uint8 step_offset;
    PASS_FAIL_TYPE retval;

    if ((retval = SettingFile__BasicLoader(SF_PTR_UI_SEQUENCER_DATA, index, &sf_loader)) == PASS)
    {
        sequence_data_ptr = (SEQUENCE_DATA_TYPE *)sf_loader.Data;       //lint !e927 cast from pointer to pointer [MISRA 2004 Rule 11.4]

        /* Copy sequence header data */
        Sequence[index].Target_GI_Address = sequence_data_ptr->GI_Address;
        Sequence[index].Target_GI_Value = sequence_data_ptr->GI_Seq_Value;
        Sequence[index].Num_Of_Steps = sequence_data_ptr->Num_Of_Steps;


        if (Sequence[index].Current_Step >= Sequence[index].Num_Of_Steps)   // See if the end of the sequence has been reached
        {
            if ((Sequence[index].Remaining_Time == 0) ||
                (SystemTimers__GetEvent(Sequence[index].Key_Press_Timeout_Handle) == TRUE))
            {
                SetTargetGI(index);
                ExecuteCallBack(index, SEQUENCE_COMPLETE);
                retval = FAIL;  // This return value is used to ensure the sequence is not continued after it is reset
            }
        }
        else    // Sequence still in progress
        {
            // Get pointer to sequence steps
            step_ptr = (uint8 *)(sequence_data_ptr + 1);        //lint !e928 cast from pointer to pointer [MISRA 2004 Rule 11.4]

            // Get displacement for the step
            step_offset = (*(step_ptr + Sequence[index].Current_Step));

            // Load data for the current step of the sequence
            if ((retval = SettingFile__BasicLoader(SF_PTR_UI_SEQUENCER_STEPS, step_offset, &sf_loader)) == PASS)
            {
            	/* Copy step header data */
                Sequence[index].Step_Data_Ptr = (SEQ_STEP_DATA_TYPE *)sf_loader.Data;       //lint !e927 cast from pointer to pointer [MISRA 2004 Rule 11.4]
                Sequence[index].Remaining_Time = Sequence[index].Step_Data_Ptr->Timer;
                Sequence[index].Num_Of_Step_Conditions = 0;
                Sequence[index].Condition_GI_Address = (GI_ADDR_TYPE *)(Sequence[index].Step_Data_Ptr + 1);     //lint !e929 cast from pointer to pointer [MISRA 2004 Rule 11.4]

                /* Find all step conditions */
                for(uint8 byte_count = 0; (byte_count < (sf_loader.Length - 2)) && (Sequence[index].Condition_GI_Address->Byte != 0xFF); byte_count++, Sequence[index].Condition_GI_Address++)
                {
                	Sequence[index].Num_Of_Step_Conditions++;
                	if(Sequence[index].Condition_GI_Address->Bits.Read_Type != HBLHMI_GI_READ_DIGITAL)
                	{
                		Sequence[index].Condition_GI_Address++;
                		byte_count++;
                	}
                }

                // Adjust pointer back to the first conditional GI Address
                Sequence[index].Condition_GI_Address = (GI_ADDR_TYPE *)(Sequence[index].Step_Data_Ptr + 1);     //lint !e929 cast from pointer to pointer [MISRA 2004 Rule 11.4]

                if (Sequence[index].Step_Data_Ptr->Timeout != 0)
                {
                    memset(&hms_timer, 0, sizeof(hms_timer));
                    hms_timer.seconds = Sequence[index].Step_Data_Ptr->Timeout;
                    SystemTimers__SetHMS(Sequence[index].Step_Timeout_Handle, &hms_timer);
                }
            }
        }
    }

    return (retval);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief   Reloads the data for a sequence from the SettingFile and resets parameters.
 */
static void ResetSequence(uint8 index)
{
    Sequence[index].Current_Step = 0;
    Sequence[index].Remaining_Time = 0;
    SystemTimers__Reset(Sequence[index].Step_Timeout_Handle);
    SystemTimers__Reset(Sequence[index].Key_Press_Timeout_Handle);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief  When a sequence is complete, the resulting/target GI is set.
 */
static void SetTargetGI(uint8 index)
{
    RegulationFilter__StopSecondFunctionTimer();
    HblHmi__SetGI((HBLHMI_GI_READ_TYPE)Sequence[index].Target_GI_Address.Bits.Read_Type, Sequence[index].Target_GI_Address.Bits.Position, Sequence[index].Target_GI_Value);
    ResetSequence(index);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief   Executes a callback when a sequence generates an event
 * @param   uint8 sequence_index
 * @param   uint8 sequence_phase
 */
static void ExecuteCallBack(uint8 sequence_index, SEQUENCE_PHASE_TYPE sequence_phase)
{
	uint8 sequence_event;

	sequence_event = ((uint8)sequence_phase << 6);
	sequence_event |= (sequence_index & 0x3F);

	Callback__Notify(&Sequence_Callback_Control_Structure, (CALLBACK_EVENT_TYPE)sequence_event);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief   Goes through all Sequences and their steps looking for the used GIs
 * @details It retrieves the used GIs for future requests
 */
static void GetGIsUsedInSequences(void)
{
    uint8 seq_index, index, gi_cond_pointer, sequence_used_gi_counter = 0;

    for (seq_index = 0; seq_index < Num_Of_Sequences; seq_index++, sequence_used_gi_counter = 0)	// For each sequence
    {
        do																							// For each step in the sequence
        {
            if (LoadStep(seq_index) == PASS)    													// Load step
            {
                for (index = 0, gi_cond_pointer = 0; index < Sequence[seq_index].Num_Of_Step_Conditions; index++, gi_cond_pointer++) // For each condition within step
                {
                    if(Sequence[seq_index].Condition_GI_Address[gi_cond_pointer].Byte != HBLHMI_GI_INVALID)	// If condition GI is valid
                    {
                        if(sequence_used_gi_counter < MAX_NUM_HANDLED_GI_IN_STEP_CONDITIONS)				// Limit condition GI in the step
                        {
                        	// Group condition GI in arrays associated with steps of sequences
                            Sequence_Conditions[seq_index].Used_GI[Sequence[seq_index].Current_Step][sequence_used_gi_counter] = Sequence[seq_index].Condition_GI_Address[gi_cond_pointer].Byte;

                            if(Sequence[seq_index].Condition_GI_Address[gi_cond_pointer].Bits.Read_Type != HBLHMI_GI_READ_DIGITAL) // For non-digital generic inputs
                            {
                            	gi_cond_pointer++;																				 // Condition pointer needs to moved by 1 additionally
                            }

                            sequence_used_gi_counter++;														// Increment number of GIs used in the step.
                        }
                        else
                        {
                            LOG_ADD_ERROR(MAX_NUM_GI_CONDITIONS_EXCEEDED, seq_index);
                            break;  																		//stop trying to add more Used GIs for this sequence
                        }
                    }
                }

                Sequence[seq_index].Current_Step++;															//Move to the next Step
                sequence_used_gi_counter = 0;
            }

            if(Sequence[seq_index].Current_Step > MAX_NUM_STEPS_IN_SEQUENCE)								//If the sequence has more than allowed steps,
            {
            	LOG_ADD_ERROR(MAX_NUM_OF_STEPS_EXCEEDED, seq_index);
            	break;	 																					//terminate loop for this sequence.
            }
        }while(Sequence[seq_index].Current_Step < Sequence[seq_index].Num_Of_Steps);
    }
}
