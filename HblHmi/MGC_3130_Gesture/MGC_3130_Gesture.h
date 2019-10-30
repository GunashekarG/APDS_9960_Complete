/*
 * MGC_3130_Gesture.h
 *
 *  Created on: 16-Aug-2019
 *      Author: GLNUG
 */

#ifndef SOURCE_XCATEGORY_HBL_HBLHMI_MGC_3130_GESTURE_MGC_3130_GESTURE_H_
#define SOURCE_XCATEGORY_HBL_HBLHMI_MGC_3130_GESTURE_MGC_3130_GESTURE_H_


#define GestureI2c_SLAVE_ADDRESS 0X42
#define Gesture_I2C_SPEED        I2C_100KHZ
#define Gesture_I2C_ENUM_TYPE    I2C1
#define Gesture_I2C_Address_Type I2C_ADDR_7BITS
#define Delay_counter_value      5000

//! define number of MGC3130 Gesture controllers.
#define Gesture_I2C_TOTAL_NUMBER_OF_SLAVE_DEVICES          1

typedef enum
{
    Gesture_I2C_IDLE,
	Gesture_I2C_ERROR,
    Gesture_I2C_BUSY,
    Gesture_I2C_IN_PROGRESS,
    Gesture_I2C_SUCCESS,
    Gesture_I2C_TIMEOUT,
    Gesture_I2C_RETRY
}Gesture_I2C_HW_STATE;

/*typedef enum
{
    Gesture_I2C_STATE_INIT,
    Gesture_I2C_STATE_READ_WRITE,
    Gesture_I2C_STATE_REQUEST_READ,
    Gesture_I2C_STATE_WAIT_WRITE_COMPLETION,
    Gesture_I2C_KEY_STATE_ACTUAL_WRITE,
    Gesture_I2C_STATE_REQUEST_DATA
}Gesture_I2C_FM_STATE_TYPE;*/

//States for state machine
/*initialization
 * write command
 * wait for write completion
 * read command
 * wait for read to complete
 * if completed read the data in the array [132 Bytes]
 * if not completed or timeout occurred, re initialize the i2c state
 * */

//Newly written ENUM's states
typedef enum{
	Gesture_I2C_STATE_INIT,
	Gesture_I2C_STATE_WRITE,
	Gesture_I2C_STATE_WAIT_WRITE_COMPLETION,
	Gesture_I2C_STATE_REQUEST_READ,
	Gesture_I2C_STATE_READ,
	Gesture_I2C_STATE_WAIT_READ_COMPLETION
}Gesture_I2C_FM_STATE_TYPE;

typedef enum
{
	Gesture_I2C_READ_COMMAND,
	Gesture_I2C_WRITE_COMMAND
}Gesture_I2C_STATE_RW_CONTROL_TYPE;

typedef PACKED struct
{
	//!Read or write I2c Operation
	Gesture_I2C_STATE_RW_CONTROL_TYPE gest_i2c_read_write;

	//! number of Read or write bytes
	    unsigned char gest_number_Of_read_write_bytes;

	//! pointer to read or write memory location
	    unsigned char gest_src_dst_buffer_ptr[200];

	//! Gesture I2C State define
	    Gesture_I2C_HW_STATE Gesture_I2C_Hw_State;

	//!If device is not responding or Busy state  wait until  this delay before initializing   I2C Bus
	    unsigned short int Gesture_I2c_Hangup_Delay_Timeout;

    //! Gesture_i2C  Finite states definition
	Gesture_I2C_FM_STATE_TYPE  Gesture_I2c_FM_State;
}Gesture_I2C_COMMAND_STRUCT;

void GestureI2c__Initialize(void);
void GestureI2c__FastHandler(void);
void GestureI2c__SlowHandler(void);
void Get_Version_info(void);


#endif /* SOURCE_XCATEGORY_HBL_HBLHMI_MGC_3130_GESTURE_MGC_3130_GESTURE_H_ */
