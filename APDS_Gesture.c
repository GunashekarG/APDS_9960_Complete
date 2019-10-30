/*
 * Apds9960.c
 *
 *  Created on: Sep 26, 2019
 *      Author: GLNUG
 */

#include "C_Extensions.h"

#include "I2c.h"
#include "Micro.h"
#include "I2cMgr.h"
#include <string.h>

#include "Apds_Gesture.h"

#define APDS_I2C_CHANNEL I2C1
#define ENABLE_REGISTER 0x80u
//#define APDS_I2C_SPEED   I2C_50KHZ

BOOL_TYPE Write_stat_enable_1 = FALSE;
BOOL_TYPE Write_stat_enable = FALSE;
BOOL_TYPE Write_stat_gmode = FALSE;
BOOL_TYPE Write_stat_gmode_1 = FALSE;
BOOL_TYPE Write_stat_gstat = FALSE;
BOOL_TYPE check_gstat = FALSE;
BOOL_TYPE check_gvalid = FALSE;
BOOL_TYPE stat_read = FALSE;
BOOL_TYPE GFLVL_read = FALSE;
BOOL_TYPE data_valid = FALSE;
BOOL_TYPE check_gvalid_fn = FALSE;
BOOL_TYPE GFIFO_U_STAT = FALSE;
BOOL_TYPE GFIFO_U_read = FALSE;
BOOL_TYPE Write_Enter_Threshold_enable_addr = FALSE;
BOOL_TYPE Write_Enter_Threshold_enable_value = FALSE;
BOOL_TYPE Read_threshold_value = FALSE;
BOOL_TYPE Write_Exit_Threshold_enable_addr = FALSE;
BOOL_TYPE Write_Exit_Threshold_enable_value = FALSE;
BOOL_TYPE Read_Exit_threshold_value = FALSE;

#define APDS_REG_CONFIG1 0x8Du
#define APDS_REG_ID 0x92u
#define APDS_I2C_ADDRESS (0x39 << 1)
#define APDS_BUF_SIZE 0x08u

//Data exclusively for Gesture
uint8 fifo_level = 0;
uint8 bytes_read = 0;
uint8 fifo_data[128];
uint8 gstatus;
int motion;
int i;

int val = 0;
static uint8 ApdsHandle;
static APDS_I2C_STATE_TYPE ApdsI2cState;
APDS_I2C_DIR_TYPE ApdsDir;
static uint8 ApdsBuf[APDS_BUF_SIZE];
static uint8 ApdsBuf_1[1];
static uint8 GmodeBuf[1];
static uint8 GstatBuf[1];
static uint8 GFLVLBuf[1];
static uint8 Threshold_Enter_Buf[1];
static uint8 Threshold_Exit_Buf[1];
//static uint8 GFIFO_U_Buf[1];
static uint8 ApdsBusyCounter;
static uint8 Enable_Value[1] = {0x41};
static uint8 GMODE_VAL[1] = {0x01};
static uint8 DEFAULT_GENTER_THRESHOLD[1] = {0x40}; 		// Threshold for entering gesture mode
static uint8 DEFAULT_GEXIT_THRESHOLD[1] = {0x30};     	// Threshold for exiting gesture mode

void Apds__Initialize(void){
	ApdsI2cState = APDS_I2C_STATE_INIT;
	memset(ApdsBuf, 0, 8);
	ApdsBusyCounter = 0;
	ApdsHandle = I2cMgr__GetDeviceHandle(APDS_I2C_CHANNEL);
}

/*
BOOL_TYPE CheckValidData(){

}
*/

void Apds__Handler(void){
    switch (ApdsI2cState){
    //setting value for ENABLE_REGISTER and reading it

    case APDS_I2C_STATE_INIT:
        if(I2cMgr__RequestBus(ApdsHandle) == TRUE){
            I2c__Initialize(APDS_I2C_CHANNEL, APDS_I2C_SPEED, I2C_ADDR_7BITS, APDS_I2C_ADDRESS);
            //I2c__Write(APDS_I2C_CHANNEL, APDS_REG_ID, ApdsBuf, 0);
            Write_stat_enable = I2c__Write(APDS_I2C_CHANNEL, ENABLE_REGISTER, Enable_Value, 1);
            //Write_stat_1 = I2c__Write(APDS_I2C_CHANNEL, ENABLE_REGISTER, ApdsBuf, 0);
            ApdsI2cState = APDS_I2C_STATE_WAIT_END_WRITE;
        }
    break;

    case APDS_I2C_STATE_WAIT_END_WRITE:
        if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_ERROR)) == I2C_ERROR_ACK){
            ApdsI2cState = APDS_I2C_STATE_INIT;
            I2cMgr__ReleaseBus(ApdsHandle);
        }
        else
        {
            if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_STATE)) == I2C_STATE_IDLE){
                ApdsI2cState = APDS_I2C_STATE_WRITE_VALUE;
                I2cMgr__ReleaseBus(ApdsHandle);
                ApdsBusyCounter = 0;
            }
            else{
            	ApdsBusyCounter++;
                if (ApdsBusyCounter == 255){
                    ApdsBusyCounter = 0;
                    ApdsI2cState = APDS_I2C_STATE_INIT;
                    I2cMgr__ReleaseBus(ApdsHandle);
                }
            }
        }
     break;

    case APDS_I2C_STATE_WRITE_VALUE:
    	Write_stat_enable_1 = I2c__Write(APDS_I2C_CHANNEL, ENABLE_REGISTER, ApdsBuf, 0);
    	ApdsI2cState = APDS_I2C_STATE_WAIT_END_WRITE_1;
    break;

    case APDS_I2C_STATE_WAIT_END_WRITE_1:
            if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_ERROR)) == I2C_ERROR_ACK){
                ApdsI2cState = APDS_I2C_STATE_INIT;
                I2cMgr__ReleaseBus(ApdsHandle);
            }
            else{
                if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_STATE)) == I2C_STATE_IDLE){
                    ApdsI2cState = APDS_I2C_STATE_REQUEST;
                    I2cMgr__ReleaseBus(ApdsHandle);
                    ApdsBusyCounter = 0;
                }
                else{
                	ApdsBusyCounter++;
                    if (ApdsBusyCounter == 255){
                        ApdsBusyCounter = 0;
                        ApdsI2cState = APDS_I2C_STATE_INIT;
                        I2cMgr__ReleaseBus(ApdsHandle);
                    }
                }
            }
    break;

    case APDS_I2C_STATE_REQUEST:
        if(I2cMgr__RequestBus(ApdsHandle) == TRUE){
            I2c__Initialize(APDS_I2C_CHANNEL, APDS_I2C_SPEED, I2C_ADDR_7BITS, APDS_I2C_ADDRESS);
            I2c__RequestRead(APDS_I2C_CHANNEL, 1);
            ApdsI2cState = APDS_I2C_STATE_CHECK_READ;
        }
    break;

    case APDS_I2C_STATE_CHECK_READ:
        if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_ERROR)) == I2C_ERROR_ACK){
            ApdsI2cState = APDS_I2C_STATE_INIT;
            I2cMgr__ReleaseBus(ApdsHandle);
        }
        else{
            if (I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_STATE) == I2C_STATE_IDLE){
                //I2c__Read(APDS_I2C_CHANNEL, ApdsBuf, 1);
            	I2c__Read(APDS_I2C_CHANNEL, ApdsBuf_1, 1);
                I2cMgr__ReleaseBus(ApdsHandle);
                ApdsI2cState = APDS_I2C_STATE_WRITE_ENTER_THRESHOLD;
                ApdsBusyCounter = 0;
            }
            else{
            	ApdsBusyCounter++;
            }
        }
     break;

   //-------------------------------------------------------------------------------------------------//

//CURRENTLY AT THE STATE OF SETTING THE ENTER AND EXIT THRESHOLD USING I2C COMMUNICATION
    case APDS_I2C_STATE_WRITE_ENTER_THRESHOLD:
    	if(I2cMgr__RequestBus(ApdsHandle) == TRUE){
    	   I2c__Initialize(APDS_I2C_CHANNEL, APDS_I2C_SPEED, I2C_ADDR_7BITS, APDS_I2C_ADDRESS);
    	   Write_Enter_Threshold_enable_addr = I2c__Write(APDS_I2C_CHANNEL, APDS9960_GPENTH, DEFAULT_GENTER_THRESHOLD, 1);
    	   ApdsI2cState = APDS_I2C_STATE_WAIT_END_WRITE_7;
    	        }
    break;

    case APDS_I2C_STATE_WAIT_END_WRITE_7:
    	if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_ERROR)) == I2C_ERROR_ACK){
    	    ApdsI2cState = APDS_I2C_STATE_INIT;
    	    I2cMgr__ReleaseBus(ApdsHandle);
    	}
    	else{
    	    if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_STATE)) == I2C_STATE_IDLE){
    	    	ApdsI2cState = APDS_I2C_STATE_WRITE_ENTER_THRESHOLD_VALUE;
    	    	I2cMgr__ReleaseBus(ApdsHandle);
    	    	ApdsBusyCounter = 0;
    	    }
    	    else{
    	         ApdsBusyCounter++;
    	         if (ApdsBusyCounter == 255){
    	             ApdsBusyCounter = 0;
    	             ApdsI2cState = APDS_I2C_STATE_INIT;
    	             I2cMgr__ReleaseBus(ApdsHandle);
    	          }
    	    }
    	}
    break;

    case APDS_I2C_STATE_WRITE_ENTER_THRESHOLD_VALUE:
    	Write_Enter_Threshold_enable_value = I2c__Write(APDS_I2C_CHANNEL, APDS9960_GPENTH, ApdsBuf, 0);
    	ApdsI2cState = APDS_I2C_STATE_WAIT_END_WRITE_8;
    break;

    case APDS_I2C_STATE_WAIT_END_WRITE_8:
    	if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_ERROR)) == I2C_ERROR_ACK){
    	    ApdsI2cState = APDS_I2C_STATE_INIT;
    	    I2cMgr__ReleaseBus(ApdsHandle);
    	}
    	else{
    	     if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_STATE)) == I2C_STATE_IDLE){
    	         ApdsI2cState = APDS_I2C_STATE_REQUEST_THRESHOLD_VALUE;
    	         I2cMgr__ReleaseBus(ApdsHandle);
    	         ApdsBusyCounter = 0;
    	     }
    	      else{
    	           ApdsBusyCounter++;
    	           if (ApdsBusyCounter == 255){
    	               ApdsBusyCounter = 0;
    	               ApdsI2cState = APDS_I2C_STATE_INIT;
    	               I2cMgr__ReleaseBus(ApdsHandle);
    	           }
    	      }
    	 }
     break;

    case APDS_I2C_STATE_REQUEST_THRESHOLD_VALUE:
    	if(I2cMgr__RequestBus(ApdsHandle) == TRUE){
    	   I2c__Initialize(APDS_I2C_CHANNEL, APDS_I2C_SPEED, I2C_ADDR_7BITS, APDS_I2C_ADDRESS);
    	   I2c__RequestRead(APDS_I2C_CHANNEL, 1);
    	   ApdsI2cState = APDS_I2C_STATE_CHECK_THRESHOLD_VALUE;
    	}
    break;

    case APDS_I2C_STATE_CHECK_THRESHOLD_VALUE:
    	if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_ERROR)) == I2C_ERROR_ACK){
    	    ApdsI2cState = APDS_I2C_STATE_INIT;
    	    I2cMgr__ReleaseBus(ApdsHandle);
    	}
    	 else{
    	      if (I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_STATE) == I2C_STATE_IDLE){
    	          //I2c__Read(APDS_I2C_CHANNEL, ApdsBuf, 1);
    	    	  Read_threshold_value = I2c__Read(APDS_I2C_CHANNEL, Threshold_Enter_Buf, 1);
    	          I2cMgr__ReleaseBus(ApdsHandle);
    	          ApdsI2cState = APDS_I2C_STATE_WRITE_EXIT_THRESHOLD;
    	          ApdsBusyCounter = 0;
    	      }
    	      else{
    	           ApdsBusyCounter++;
    	      }
    	  }
     break;

    //--------------------------------------------------------------------------------------------------//
    //CURRENTLY AT THE STATE OF SETTING THE ENTER AND EXIT THRESHOLD USING I2C COMMUNICATION
        case APDS_I2C_STATE_WRITE_EXIT_THRESHOLD:
        	if(I2cMgr__RequestBus(ApdsHandle) == TRUE){
        	   I2c__Initialize(APDS_I2C_CHANNEL, APDS_I2C_SPEED, I2C_ADDR_7BITS, APDS_I2C_ADDRESS);
        	   Write_Exit_Threshold_enable_addr = I2c__Write(APDS_I2C_CHANNEL, APDS9960_GEXTH, DEFAULT_GEXIT_THRESHOLD, 1);
        	   ApdsI2cState = APDS_I2C_STATE_WAIT_END_WRITE_9;
        	}
        break;

        case APDS_I2C_STATE_WAIT_END_WRITE_9:
        	if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_ERROR)) == I2C_ERROR_ACK){
        	    ApdsI2cState = APDS_I2C_STATE_INIT;
        	    I2cMgr__ReleaseBus(ApdsHandle);
        	}
        	else{
        	    if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_STATE)) == I2C_STATE_IDLE){
        	    	ApdsI2cState = APDS_I2C_STATE_WRITE_EXIT_THRESHOLD_VALUE;
        	    	I2cMgr__ReleaseBus(ApdsHandle);
        	    	ApdsBusyCounter = 0;
        	    }
        	    else{
        	         ApdsBusyCounter++;
        	         if (ApdsBusyCounter == 255){
        	             ApdsBusyCounter = 0;
        	             ApdsI2cState = APDS_I2C_STATE_INIT;
        	             I2cMgr__ReleaseBus(ApdsHandle);
        	          }
        	    }
        	}
        break;

        case APDS_I2C_STATE_WRITE_EXIT_THRESHOLD_VALUE:
        	Write_Exit_Threshold_enable_value = I2c__Write(APDS_I2C_CHANNEL, APDS9960_GEXTH, ApdsBuf, 0);
        	ApdsI2cState = APDS_I2C_STATE_WAIT_END_WRITE_10;
        break;

        case APDS_I2C_STATE_WAIT_END_WRITE_10:
        	if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_ERROR)) == I2C_ERROR_ACK){
        	    ApdsI2cState = APDS_I2C_STATE_INIT;
        	    I2cMgr__ReleaseBus(ApdsHandle);
        	}
        	else{
        	     if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_STATE)) == I2C_STATE_IDLE){
        	         ApdsI2cState = APDS_I2C_STATE_REQUEST_EXIT_THRESHOLD_VALUE;
        	         I2cMgr__ReleaseBus(ApdsHandle);
        	         ApdsBusyCounter = 0;
        	     }
        	      else{
        	           ApdsBusyCounter++;
        	           if (ApdsBusyCounter == 255){
        	               ApdsBusyCounter = 0;
        	               ApdsI2cState = APDS_I2C_STATE_INIT;
        	               I2cMgr__ReleaseBus(ApdsHandle);
        	           }
        	      }
        	 }
         break;

        case APDS_I2C_STATE_REQUEST_EXIT_THRESHOLD_VALUE:
        	if(I2cMgr__RequestBus(ApdsHandle) == TRUE){
        	   I2c__Initialize(APDS_I2C_CHANNEL, APDS_I2C_SPEED, I2C_ADDR_7BITS, APDS_I2C_ADDRESS);
        	   I2c__RequestRead(APDS_I2C_CHANNEL, 1);
        	   ApdsI2cState = APDS_I2C_STATE_CHECK_EXIT_THRESHOLD_VALUE;
        	}
        break;

        case APDS_I2C_STATE_CHECK_EXIT_THRESHOLD_VALUE:
        	if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_ERROR)) == I2C_ERROR_ACK){
        	    ApdsI2cState = APDS_I2C_STATE_INIT;
        	    I2cMgr__ReleaseBus(ApdsHandle);
        	}
        	 else{
        	      if (I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_STATE) == I2C_STATE_IDLE){
        	          //I2c__Read(APDS_I2C_CHANNEL, ApdsBuf, 1);
        	    	  Read_Exit_threshold_value = I2c__Read(APDS_I2C_CHANNEL, Threshold_Exit_Buf, 1);
        	          I2cMgr__ReleaseBus(ApdsHandle);
        	          ApdsI2cState = APDS_I2C_GMODE_ENABLE;
        	          ApdsBusyCounter = 0;
        	      }
        	      else{
        	           ApdsBusyCounter++;
        	      }
        	  }
        break;
    //--------------------------------------------------------------------------------------------------//
        //GMODE_ENABLE_LINK
        //setting value for GMODE REGISTER and reading it
        case APDS_I2C_GMODE_ENABLE:
        	if(I2cMgr__RequestBus(ApdsHandle) == TRUE){
        		I2c__Initialize(APDS_I2C_CHANNEL, APDS_I2C_SPEED, I2C_ADDR_7BITS, APDS_I2C_ADDRESS);
        		Write_stat_gmode = I2c__Write(APDS_I2C_CHANNEL, APDS9960_GCONF4, GMODE_VAL, 1);
        		ApdsI2cState = APDS_I2C_STATE_WAIT_END_WRITE_2;
        	}
       	break;

       case APDS_I2C_STATE_WAIT_END_WRITE_2:
            if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_ERROR)) == I2C_ERROR_ACK){
            ApdsI2cState = APDS_I2C_STATE_INIT;
            I2cMgr__ReleaseBus(ApdsHandle);
            }
            else{
                 if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_STATE)) == I2C_STATE_IDLE){
                     ApdsI2cState = APDS_I2C_GMODE_ENABLE_CHECK;
                     I2cMgr__ReleaseBus(ApdsHandle);
                     ApdsBusyCounter = 0;
                 }
                 else{
                      ApdsBusyCounter++;
                      if (ApdsBusyCounter == 255){
                          ApdsBusyCounter = 0;
                          ApdsI2cState = APDS_I2C_STATE_INIT;
                          I2cMgr__ReleaseBus(ApdsHandle);
                          }
                      }
                  }
       break;

       case APDS_I2C_GMODE_ENABLE_CHECK:
    	   	Write_stat_gmode_1 = I2c__Write(APDS_I2C_CHANNEL, APDS9960_GCONF4, ApdsBuf, 0);
       	    ApdsI2cState = APDS_I2C_STATE_WAIT_END_WRITE_3;
       break;

       case APDS_I2C_STATE_WAIT_END_WRITE_3:
            if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_ERROR)) == I2C_ERROR_ACK){
            	ApdsI2cState = APDS_I2C_STATE_INIT;
            	I2cMgr__ReleaseBus(ApdsHandle);
            }
            else{
                 if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_STATE)) == I2C_STATE_IDLE){
                     ApdsI2cState = APDS_I2C_STATE_REQUEST_GMODE;
                     I2cMgr__ReleaseBus(ApdsHandle);
                     ApdsBusyCounter = 0;
                 }
                 else{
                      ApdsBusyCounter++;
                      if (ApdsBusyCounter == 255){
                          ApdsBusyCounter = 0;
                          ApdsI2cState = APDS_I2C_STATE_INIT;
                          I2cMgr__ReleaseBus(ApdsHandle);
                      }
                 }
            }
        break;

       case APDS_I2C_STATE_REQUEST_GMODE:
    	   if(I2cMgr__RequestBus(ApdsHandle) == TRUE){
    	              I2c__Initialize(APDS_I2C_CHANNEL, APDS_I2C_SPEED, I2C_ADDR_7BITS, APDS_I2C_ADDRESS);
    	              I2c__RequestRead(APDS_I2C_CHANNEL, 1);
    	              ApdsI2cState = APDS_I2C_STATE_CHECK_READ_GMODE;
    	          }
        break;

       case APDS_I2C_STATE_CHECK_READ_GMODE:
    	   if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_ERROR)) == I2C_ERROR_ACK){
    	      ApdsI2cState = APDS_I2C_STATE_INIT;
    	      I2cMgr__ReleaseBus(ApdsHandle);
    	   }
    	   else{
    	       if (I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_STATE) == I2C_STATE_IDLE){
    	          //I2c__Read(APDS_I2C_CHANNEL, ApdsBuf, 1);
    	          I2c__Read(APDS_I2C_CHANNEL, GmodeBuf, 1);
    	          I2cMgr__ReleaseBus(ApdsHandle);
    	          ApdsI2cState = APDS_I2C_STATE_GSTAT;
    	          ApdsBusyCounter = 0;
    	        }
    	        else{
    	            ApdsBusyCounter++;
    	        }
    	   }
    	break;

   //-------------------------------------------------------------------------------------------------//
   //Reading the value from GSTATUS register
       case APDS_I2C_STATE_GSTAT:
               if(I2cMgr__RequestBus(ApdsHandle) == TRUE){
                  I2c__Initialize(APDS_I2C_CHANNEL, APDS_I2C_SPEED, I2C_ADDR_7BITS, APDS_I2C_ADDRESS);
                  Write_stat_gstat = I2c__Write(APDS_I2C_CHANNEL, APDS9960_GSTATUS, ApdsBuf, 0);
                  ApdsI2cState = APDS_I2C_STATE_WAIT_END_WRITE_4;
               }
       break;

       case APDS_I2C_STATE_WAIT_END_WRITE_4:
            if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_ERROR)) == I2C_ERROR_ACK){
                ApdsI2cState = APDS_I2C_STATE_INIT;
                I2cMgr__ReleaseBus(ApdsHandle);
            }
            else{
                if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_STATE)) == I2C_STATE_IDLE){
                    ApdsI2cState = APDS_I2C_STATE_REQUEST_GSTAT;
                    I2cMgr__ReleaseBus(ApdsHandle);
                    ApdsBusyCounter = 0;
                }
                 else{
                      ApdsBusyCounter++;
                      if (ApdsBusyCounter == 255){
                         ApdsBusyCounter = 0;
                         ApdsI2cState = APDS_I2C_STATE_INIT;
                         I2cMgr__ReleaseBus(ApdsHandle);
                      }
                 }
             }
        break;

       case APDS_I2C_STATE_REQUEST_GSTAT:
    	   if(I2cMgr__RequestBus(ApdsHandle) == TRUE){
    	      I2c__Initialize(APDS_I2C_CHANNEL, APDS_I2C_SPEED, I2C_ADDR_7BITS, APDS_I2C_ADDRESS);
    	      I2c__RequestRead(APDS_I2C_CHANNEL, 1);
    	      ApdsI2cState = APDS_I2C_STATE_CHECK_READ_GSTAT;
    	   }
       break;


       case APDS_I2C_STATE_CHECK_READ_GSTAT:
    	   if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_ERROR)) == I2C_ERROR_ACK){
    	       ApdsI2cState = APDS_I2C_STATE_INIT;
    	       I2cMgr__ReleaseBus(ApdsHandle);
    	   }
    	   else{
    	       if (I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_STATE) == I2C_STATE_IDLE){
    	       	   //I2c__Read(APDS_I2C_CHANNEL, ApdsBuf, 1);
    	       	   stat_read = I2c__Read(APDS_I2C_CHANNEL, GstatBuf, 1);
    	       	   I2cMgr__ReleaseBus(ApdsHandle);
    	       	   val = GstatBuf[0];
    	       	   val &= APDS9960_GVALID_1;
    	       	   if(val == 1){
    	       	     check_gstat = TRUE;
    	       	   }
    	       	   ApdsI2cState = APDS_I2C_STATE_READ_GESTURE;
    	       	   ApdsBusyCounter = 0;
    	       	}
    	       	else{
    	       	    ApdsBusyCounter++;
    	       	}
    	    }
        break;

       case APDS_I2C_STATE_READ_GESTURE:
    	   /* Make sure that power and gesture is on and data is valid */
    	      if( check_gstat == TRUE && (ApdsBuf_1[0] & APDS9960_VALID_CHECK) ) {
    	    	  ApdsDir =  DIR_NONE;
    	      }
    	      if(check_gstat == TRUE){
    	          check_gvalid = TRUE;
    	      }
    	      if(stat_read == TRUE){
    	       	  data_valid = TRUE;
    	      }else{
    	      	  data_valid = FALSE;
    	      }
    	      //  If we have valid data, read in FIFO
    	      if( (GstatBuf[0] & APDS9960_GVALID_1) == APDS9960_GVALID_1 ){
    	      // Read the current FIFO level (i.e., read the contents of APDS9960_GFLVL)
    	      if(I2cMgr__RequestBus(ApdsHandle) == TRUE){
    	         I2c__Initialize(APDS_I2C_CHANNEL, APDS_I2C_SPEED, I2C_ADDR_7BITS, APDS_I2C_ADDRESS);
    	         Write_stat_gstat = I2c__Write(APDS_I2C_CHANNEL, APDS9960_GFLVL, ApdsBuf, 0);
    	         ApdsI2cState = APDS_I2C_STATE_WAIT_END_WRITE_5;
    	      }
    	         }
    	      //  If we have valid data, read in FIFO
    	    //  ApdsI2cState = APDS_I2C_STATE_IDLE;
    	      /* Keep looping as long as gesture data is valid */
    	      /*while(1){
    	    	  // Wait some time to collect next batch of FIFO data
    	    	  Micro__DelayNumNops(1000);
    	    	  if(check_gstat == TRUE){
    	    		  check_gvalid = TRUE;
    	    	  }

    	    	  // Get the contents of the STATUS register. Is data still valid?
    	    	   if(stat_read == TRUE){
    	    		  data_valid = TRUE;
    	    	  }else{
    	    		  data_valid = FALSE;
    	    	  }
    	      }*/
    	      break;

    	    	 //  If we have valid data, read in FIFO
    	    	   	   //CURRENT UPDATE...WAS TRYING TO CREATE THE FUNCTION WITH PARAMETERS
    	    	 //  check_gvalid_fn = CheckValidData();
    	    	   /* if( (GstatBuf[0] & APDS9960_GVALID_1) == APDS9960_GVALID_1 ){
    	    		  // Read the current FIFO level (i.e., read the contents of APDS9960_GFLVL)
    	    		  if(I2cMgr__RequestBus(ApdsHandle) == TRUE)
    	    		                 {
    	    		                     I2c__Initialize(APDS_I2C_CHANNEL, APDS_I2C_SPEED, I2C_ADDR_7BITS, APDS_I2C_ADDRESS);
    	    		                     Write_stat_gstat = I2c__Write(APDS_I2C_CHANNEL, APDS9960_GFLVL, ApdsBuf, 0);
    	    		                     ApdsI2cState = APDS_I2C_STATE_WAIT_END_WRITE_5;
    	    		                 }
    	    	  }
    	      }
    	   break;*/


       case APDS_I2C_STATE_WAIT_END_WRITE_5:
    	   if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_ERROR)) == I2C_ERROR_ACK){
    	      ApdsI2cState = APDS_I2C_STATE_INIT;
    	      I2cMgr__ReleaseBus(ApdsHandle);
    	   }
    	   else{
    	        if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_STATE)) == I2C_STATE_IDLE){
    	           ApdsI2cState = APDS_I2C_STATE_REQUEST_GFLVL;
    	           I2cMgr__ReleaseBus(ApdsHandle);
    	           ApdsBusyCounter = 0;
    	        }
    	        else{
    	            ApdsBusyCounter++;
    	            if (ApdsBusyCounter == 255){
    	               ApdsBusyCounter = 0;
    	               ApdsI2cState = APDS_I2C_STATE_INIT;
    	               I2cMgr__ReleaseBus(ApdsHandle);
    	            }
    	        }
    	    }
       break;

    //-------------------------------------------------------------------------------------------------//

       case APDS_I2C_STATE_REQUEST_GFLVL:
    	   if(I2cMgr__RequestBus(ApdsHandle) == TRUE){
    	      I2c__Initialize(APDS_I2C_CHANNEL, APDS_I2C_SPEED, I2C_ADDR_7BITS, APDS_I2C_ADDRESS);
    	      I2c__RequestRead(APDS_I2C_CHANNEL, 1);
    	      ApdsI2cState = APDS_I2C_STATE_CHECK_READ_GFLVL;
    	   }
    	   break;

    	    case APDS_I2C_STATE_CHECK_READ_GFLVL:
    	   if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_ERROR)) == I2C_ERROR_ACK){
    	       ApdsI2cState = APDS_I2C_STATE_INIT;
    	       I2cMgr__ReleaseBus(ApdsHandle);
    	   }
    	   else{
    	       if (I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_STATE) == I2C_STATE_IDLE){
    	       	   GFLVL_read = I2c__Read(APDS_I2C_CHANNEL, GFLVLBuf, 1);
    	       	   I2cMgr__ReleaseBus(ApdsHandle);
    	       	   if(GFLVLBuf[0] > 0){
    	       	      //read the contents of APDS9960_GFIFO_U into fifo_data that's declared above
    	       	      ApdsI2cState = APDS_I2C_STATE_GFIFO_U;
    	       	    }

    	       	    ApdsBusyCounter = 0;
    	       	}
    	       	else{
    	       	     ApdsBusyCounter++;
    	       	}
    	    }
    	   break;
    	   /* If there's stuff in the FIFO, read it into our data block */
    	  /* if(GFLVLBuf[0] > 0){
    		   //read the contents of APDS9960_GFIFO_U into fifo_data that's declared above
    	   }*/
    case APDS_I2C_STATE_GFIFO_U:
    	 if(I2cMgr__RequestBus(ApdsHandle) == TRUE){
    	    I2c__Initialize(APDS_I2C_CHANNEL, APDS_I2C_SPEED, I2C_ADDR_7BITS, APDS_I2C_ADDRESS);
    	    GFIFO_U_STAT = I2c__Write(APDS_I2C_CHANNEL, APDS9960_GFIFO_U, ApdsBuf, 0);
    	    //Write_stat_1 = I2c__Write(APDS_I2C_CHANNEL, ENABLE_REGISTER, ApdsBuf, 0);
    	    ApdsI2cState = APDS_I2C_STATE_WAIT_END_WRITE_6;
    	 }
    break;

    case APDS_I2C_STATE_WAIT_END_WRITE_6:
    	if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_ERROR)) == I2C_ERROR_ACK){
    	    ApdsI2cState = APDS_I2C_STATE_INIT;
    	    I2cMgr__ReleaseBus(ApdsHandle);
    	}
    	else{
    	    if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_STATE)) == I2C_STATE_IDLE){
    	    	ApdsI2cState = APDS_I2C_STATE_REQUEST_GFIFO_U;
    	    	I2cMgr__ReleaseBus(ApdsHandle);
    	    	ApdsBusyCounter = 0;
    	    }
    	    else{
    	    	ApdsBusyCounter++;
    	    	if (ApdsBusyCounter == 255){
    	    	    ApdsBusyCounter = 0;
    	    	    ApdsI2cState = APDS_I2C_STATE_INIT;
    	    	    I2cMgr__ReleaseBus(ApdsHandle);
    	    	}
    	    }
    	}
    break;

    case APDS_I2C_STATE_REQUEST_GFIFO_U:
    	if(I2cMgr__RequestBus(ApdsHandle) == TRUE){
    	   I2c__Initialize(APDS_I2C_CHANNEL, APDS_I2C_SPEED, I2C_ADDR_7BITS, APDS_I2C_ADDRESS);
    	   I2c__RequestRead(APDS_I2C_CHANNEL, 1);
    	   ApdsI2cState = APDS_I2C_STATE_CHECK_READ_GFIFO_U;
    	}
    break;

    case APDS_I2C_STATE_CHECK_READ_GFIFO_U:
    	 if ((I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_ERROR)) == I2C_ERROR_ACK){
    	      ApdsI2cState = APDS_I2C_STATE_INIT;
    	      I2cMgr__ReleaseBus(ApdsHandle);
    	 }
    	 else{
    		 if (I2c__GetStatus(APDS_I2C_CHANNEL, I2C_STATUS_STATE) == I2C_STATE_IDLE){
    			 GFIFO_U_read = I2c__Read(APDS_I2C_CHANNEL, fifo_data, 128);
    			 I2cMgr__ReleaseBus(ApdsHandle);
    			 ApdsI2cState = APDS_I2C_STATE_IDLE;
    			 ApdsBusyCounter = 0;
    		 }
    	     else{
    	    	 ApdsBusyCounter++;
    	     }
    	 }
    break;
    //-------------------------------------------------------------------------------------------------//
    case APDS_I2C_STATE_IDLE:

    break;
    //-------------------------------------------------------------------------------------------------//
    default: break;
    }
}
