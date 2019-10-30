/*
 * MGC_3130_Gesture.c
 *
 *  Created on: 16-Aug-2019
 *      Author: GLNUG
 */


#include "I2c.h"
#include "Gpio.h"
#include "stdio.h"
#include "MGC_3130_Gesture.h"
#include "Micro.h"

//I2CMGR Code Added
#include "I2cMgr.h"

//! This is used to delay first I2C transaction  after device boot up
//! Delay in seconds = Gesture_I2C_STARTUP_DELAY_TIMEOUT * rate at which I2C handler is called
#ifndef Gesture_I2C_STARTUP_DELAY_TIMEOUT
#define Gesture_I2C_STARTUP_DELAY_TIMEOUT  			1000
#endif

//! If device is not responding or in busy state wait until this delay before initializing I2C Bus
//! Delay in seconds = Gesture_I2C_HANGUP_DELAY_TIMEOUT * rate at which I2C handler is called
#ifndef Gesture_I2C_HANGUP_DELAY_TIMEOUT
#define Gesture_I2C_HANGUP_DELAY_TIMEOUT   			100
#endif

static unsigned int Gesture_I2c_Startup_delay_timeout = Gesture_I2C_STARTUP_DELAY_TIMEOUT;

//Added code for testing I2C communication using I2CMgr
typedef struct _MGC3130_DEVICE_TYPE
{
    I2C_ENUM_TYPE i2c_channel;
    uint8 i2c_address;
}MGC3130_DEVICE_TYPE;

typedef enum _MGC3130_STATE_TYPE
{
    MGC3130_STATE_IDLE,
    MGC3130_STATE_WRITE,
    MGC3130_STATE_REQUEST_READ,
    MGC3130_STATE_READ,
} MGC3130_STATE_TYPE;

typedef enum _MGC3130_STATUS_TYPE
{
    MGC3130_STATUS_BUSY,
    MGC3130_STATUS_FREE,
    MGC3130_STATUS_ERROR
} MGC3130_STATUS_TYPE;

static const MGC3130_DEVICE_TYPE device;
static MGC3130_STATE_TYPE state;
static uint8 device_handle;

static Gesture_I2C_HW_STATE Gesture_I2c_StateMachine(Gesture_I2C_COMMAND_STRUCT *Gesture_I2c_Datastruct);

//Global variables declared for testing purpose
		BOOL_TYPE status_write = FALSE, status_request_read = FALSE, status_read = FALSE;
		uint8 write_buffer[12] = {0x0C, 0x00, 0x00, 0x06, 0x83, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //DATA TO BE WRITTEN TO GET THE VERSION INFO OF GestIC
		uint8 read_buffer[132];
		uint16 write_buffer_size = 12, read_buffer_size = 132;
		BOOL_TYPE check_status = FALSE;
		BOOL_TYPE init_status;
		int Return_datastruct;
		Gesture_I2C_COMMAND_STRUCT GestIC_Struct_obj;
		//yet to initialize the parameters for the above object

		//Added for I2CMgr checking
		BOOL_TYPE ReqBus_status = FALSE, ReleaseBus_status = FALSE;

/*
void GestureI2c__Initialize(void){
	//	BOOL_TYPE init_status;
	I2c__Clear(Gesture_I2C_ENUM_TYPE);
	init_status = I2c__Initialize(Gesture_I2C_ENUM_TYPE , Gesture_I2C_SPEED, I2C_ADDR_7BITS, GestureI2c_SLAVE_ADDRESS);
	//Get_Version_info();
}
*/
void GestureI2c__Initialize(void){
	//I2CMGR Code Added
	I2cMgr__Initialize();
	device_handle = 0;
	device_handle = I2cMgr__GetDeviceHandle(Gesture_I2C_ENUM_TYPE);
	state = MGC3130_STATE_IDLE;
}


void GestureI2c__FastHandler(void){

}

/*
void Get_Version_info(void){
	BOOL_TYPE status_write = FALSE, status_request_read = FALSE, status_read = FALSE;
		uint8 write_buffer[12] = {0x0C, 0x00, 0x00, 0x06, 0x83, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //DATA TO BE WRITTEN TO GET THE VERSION INFO OF GestIC
		uint8 read_buffer[132];
		uint16 write_buffer_size = 12, read_buffer_size = 132;
		status_write = I2c__Write(Gesture_I2C_ENUM_TYPE, GestureI2c_SLAVE_ADDRESS, write_buffer,write_buffer_size);
		Micro__DelayNumNops(Delay_counter_value);
		if(status_write == TRUE){
			// Micro__DelayNumNops(Dleay_counter_value);
			status_request_read = I2c__RequestRead(Gesture_I2C_ENUM_TYPE,read_buffer_size);
			if(status_request_read == TRUE){
				status_read	= I2c__Read(Gesture_I2C_ENUM_TYPE, read_buffer,read_buffer_size);
				if(status_read == TRUE){
					//If entered Read successful
					printf("%s","Version Info Successfully obtained");
				}
			}
		}
}
*/

//Static function for state machine
/*
static Gesture_I2C_HW_STATE Gesture_I2c_StateMachine(Gesture_I2C_COMMAND_STRUCT *Gesture_I2c_Datastruct){
		unsigned short response;
	   // Get the current I2C state before initiating a transaction, Response can be I2C_STATE_BUSY or no busy
	        response = I2c__GetStatus(Gesture_I2C_ENUM_TYPE, I2C_STATUS_STATE);

	        if (response == I2C_STATE_BUSY){
	        	Gesture_I2c_Datastruct->Gesture_I2c_Hangup_Delay_Timeout++;

	        	//! If slave is busy for longer duration, reinitialize I2C communication to recover from any errors
	        	if (Gesture_I2c_Datastruct->Gesture_I2c_Hangup_Delay_Timeout >= Gesture_I2C_HANGUP_DELAY_TIMEOUT){
	        		I2c__Clear(I2C0);

	        		//! Re-init I2C bus for next transaction
	        		 I2c__Initialize(Gesture_I2C_ENUM_TYPE , Gesture_I2C_SPEED, I2C_ADDR_7BITS, GestureI2c_SLAVE_ADDRESS);
	        		 //! Re-init all the timeout variables
	        					Gesture_I2c_Datastruct->Gesture_I2c_Hangup_Delay_Timeout = 0;
	        					Gesture_I2c_Datastruct->Gesture_I2c_FM_State = Gesture_I2C_STATE_INIT;
	        					Gesture_I2c_Datastruct->Gesture_I2C_Hw_State = Gesture_I2C_TIMEOUT;
	        		            return Gesture_I2c_Datastruct->Gesture_I2C_Hw_State;
	        		  }
	        	 //! I2C is busy, but timeout has not occurred. Initiate retry
	        	 else{
	        	       Gesture_I2c_Datastruct->Gesture_I2C_Hw_State = Gesture_I2C_RETRY;
	        	       return Gesture_I2C_RETRY;
	        	      }
	        	}

	        //! Get the current I2C status i.e. ACK/NACK is received
	            response = I2c__GetStatus(Gesture_I2C_ENUM_TYPE, I2C_STATUS_ERROR);

	            //! if an error is detected in I2C communication, re-init I2C BUS
	               if (response != I2C_ERROR_NONE)
	               {
	                   I2c__Clear(I2C0);

	                   //! Re-init I2C bus for next transaction
		        		response = I2c__Initialize(Gesture_I2C_ENUM_TYPE , Gesture_I2C_SPEED, I2C_ADDR_7BITS, GestureI2c_SLAVE_ADDRESS);

	                   //! Re-init I2C state
		        		Gesture_I2c_Datastruct->Gesture_I2c_FM_State = Gesture_I2C_STATE_INIT;
		        		Gesture_I2c_Datastruct->Gesture_I2C_Hw_State = Gesture_I2C_ERROR;
	                    return Gesture_I2c_Datastruct->Gesture_I2C_Hw_State;
	               }

	               switch (Gesture_I2c_Datastruct->Gesture_I2c_FM_State){

	               case Gesture_I2C_STATE_INIT:
	            	   I2c__Clear(Gesture_I2C_ENUM_TYPE);
	            	   Gesture_I2c_Datastruct->Gesture_I2c_Hangup_Delay_Timeout = 0;
	            	   response = I2c__Initialize(Gesture_I2C_ENUM_TYPE , Gesture_I2C_SPEED, I2C_ADDR_7BITS, GestureI2c_SLAVE_ADDRESS);
	            	   Gesture_I2c_Datastruct->Gesture_I2C_Hw_State = Gesture_I2C_IDLE;
	            	   Gesture_I2c_Datastruct->Gesture_I2c_FM_State = Gesture_I2C_STATE_WRITE;
	            	   break;

	               case Gesture_I2C_STATE_WRITE:
	            	   	   //Write command
	            	   status_write = I2c__Write(Gesture_I2C_ENUM_TYPE, GestureI2c_SLAVE_ADDRESS, write_buffer,write_buffer_size);
	            	   Gesture_I2c_Datastruct->Gesture_I2C_Hw_State = Gesture_I2C_IDLE;
	            	   Gesture_I2c_Datastruct->Gesture_I2c_FM_State = Gesture_I2C_STATE_REQUEST_READ;

	            	   break;


	               case Gesture_I2C_STATE_WAIT_WRITE_COMPLETION:
	            	   	   //wait for completion
		                Gesture_I2c_Datastruct->Gesture_I2C_Hw_State = Gesture_I2C_IN_PROGRESS;
	            	   	Gesture_I2c_Datastruct->Gesture_I2c_FM_State = Gesture_I2C_STATE_REQUEST_READ;
	            	   break;

				   case Gesture_I2C_STATE_REQUEST_READ:
				       status_request_read = I2c__RequestRead(Gesture_I2C_ENUM_TYPE,read_buffer_size);
	            	   Gesture_I2c_Datastruct->Gesture_I2c_FM_State = Gesture_I2C_STATE_READ;
	            	   Gesture_I2c_Datastruct->Gesture_I2C_Hw_State = Gesture_I2C_IN_PROGRESS;
	            	   break;

				   case Gesture_I2C_STATE_READ:
					   status_read = I2c__Read(Gesture_I2C_ENUM_TYPE, read_buffer,read_buffer_size);
					   if(status_read == TRUE){
						   Gesture_I2c_Datastruct->Gesture_I2c_FM_State = Gesture_I2C_STATE_INIT;
						   Gesture_I2c_Datastruct->Gesture_I2C_Hw_State = Gesture_I2C_IDLE;
					   }

	            	   break;

				   default:
	            	   Gesture_I2c_Datastruct->Gesture_I2c_FM_State = Gesture_I2C_STATE_INIT;
	            	   Gesture_I2c_Datastruct->Gesture_I2C_Hw_State = Gesture_I2C_IDLE;
	            	   break;
	               }

	               return Gesture_I2c_Datastruct->Gesture_I2C_Hw_State;
	        }
*/

/*
void GestureI2c__SlowHandler(void){
	Return_datastruct = Gesture_I2c_StateMachine(&GestIC_Struct_obj);
}
*/

//Added handler code for testing using I2CMgr
MGC3130_STATUS_TYPE Mgc3130__Handler(void )
{
    MGC3130_STATUS_TYPE response = MGC3130_STATUS_BUSY;

    switch (state)
    {
         case MGC3130_STATE_IDLE:
			if(I2cMgr__RequestBus(device_handle) == TRUE)
			{
				I2c__Initialize(Gesture_I2C_ENUM_TYPE , Gesture_I2C_SPEED, I2C_ADDR_7BITS, GestureI2c_SLAVE_ADDRESS);
				state = MGC3130_STATE_WRITE;
			}
            break;
        case MGC3130_STATE_WRITE:
            if (I2c__GetStatus(Gesture_I2C_ENUM_TYPE, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                // perform the write
            	status_write = I2c__Write(Gesture_I2C_ENUM_TYPE, GestureI2c_SLAVE_ADDRESS, write_buffer,write_buffer_size);
            	state = MGC3130_STATE_REQUEST_READ;
            }
            break;

        case MGC3130_STATE_REQUEST_READ:
            if (I2c__GetStatus(Gesture_I2C_ENUM_TYPE, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                if(I2cMgr__RequestBus(device_handle) == TRUE)
                {
                	I2c__Initialize(Gesture_I2C_ENUM_TYPE , Gesture_I2C_SPEED, I2C_ADDR_7BITS, GestureI2c_SLAVE_ADDRESS);
                	status_request_read = I2c__RequestRead(Gesture_I2C_ENUM_TYPE,read_buffer_size);
                    state = MGC3130_STATE_READ;
                }
            }
            break;
        case MGC3130_STATE_READ:
            if (I2c__GetStatus(Gesture_I2C_ENUM_TYPE, I2C_STATUS_STATE) == I2C_ERROR_ACK)
            {
            	status_read = I2c__Read(Gesture_I2C_ENUM_TYPE, read_buffer,read_buffer_size);;
BOOL_TYPE init_stat = FALSE, write_stat = FALSE;            	response = MGC3130_STATUS_ERROR;
            	state = MGC3130_STATE_IDLE;
            }
            if (I2c__GetStatus(Gesture_I2C_ENUM_TYPE, I2C_STATUS_STATE) == I2C_STATE_IDLE)
            {
                // perform the Read
            	I2c__Read(Gesture_I2C_ENUM_TYPE, read_buffer,read_buffer_size);
                state = MGC3130_STATE_IDLE; //After completion bus will go idle
            }
            I2cMgr__ReleaseBus(device_handle);
            break;

        default:
            state = MGC3130_STATE_IDLE;
            response = MGC3130_STATUS_FREE;
            break;
    }

    return(response);
}

//Added code for testing using I2CMgr
void GestureI2c__SlowHandler(void){
	I2cMgr__Handler();
	Mgc3130__Handler();
	//Return_datastruct = Gesture_I2c_StateMachine(&GestIC_Struct_obj);
}






