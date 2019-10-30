/**
 *  @file       
 *
 *  @brief      Standard API for OnSemi CAT9532 LED Driver over I2c
 *
 *
 *  $Header: Led9532.h 1.3 2014/10/06 01:01:33EDT Arun Raj R (rar) Exp  $
 *
 *  @copyright  Copyright 2013-$Date: 2014/10/06 01:01:33EDT $. Whirlpool Corporation. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------
#ifndef LED9532_H_
#define LED9532_H_

#include "SystemConfig.h"

#if (LED_DRIVER_CAT9532 == ENABLED)
#include "I2c.h"

//=====================================================================================================================
//-------------------------------------- PUBLIC (Extern Variables, Constants & Defines) -------------------------------
//=====================================================================================================================

#define     LED9532_NUM_LEDS                16
#define     LED9532_WRITE_REGISTER_ADDRESS  0X12
#define     LED9532_READ_REGISTER_ADDRESS   0X10

typedef enum _LED9532_STATUS_TYPE
{
    LED9532_STATUS_BUSY,
    LED9532_STATUS_FREE,
    LED9532_STATUS_ERROR
} LED9532_STATUS_TYPE;

typedef enum _LED9532_PWM_TYPE
{
    LED9532_PWM0,
    LED9532_PWM1,
    NUM_LED9532_PWM
} LED9532_PWM_TYPE;

typedef enum _LED9532_LED_TYPE
{
    LED9532_LED_OFF = 0,
    LED9532_LED_ON  =1,
    LED9532_LED_PWM0 =2,
    LED9532_LED_PWM1 = 3
} LED9532_LED_TYPE;

typedef enum _LED9532_STATE_TYPE
{
    LED9532_STATE_IDLE,
    LED9532_STATE_CONFIG,
    LED9532_STATE_WRITE,
    LED9532_STATE_WAIT_WRITE_END,
    LED9532_READ_INITIALIZE,
    LED9532_STATE_READ,
    LED9532_STATE_WAIT_READ_END
} LED9532_STATE_TYPE;

typedef enum _LED9532_READ_STATUS_TYPE
{
    LED9532_READ_NOT_STARTED,
    LED9532_READ_IN_PROGRESS,
    LED9532_READ_COMPLETED,
    LED9532_READ_ERROR
} LED9532_READ_STATUS_TYPE;

typedef enum _LED9532_INPUT_TYPE
{
    LED9532_INPUT_LOW,
    LED9532_INPUT_HIGH,
    LED9532_INPUT_INVALID
} LED9532_INPUT_TYPE;

typedef struct _LED9532_DEVICE_TYPE
{
    I2C_ENUM_TYPE i2c_channel;
    uint8 i2c_address;
}LED9532_DEVICE_TYPE;

typedef struct
{
    uint8 psc0;
    uint8 pwm0;
    uint8 psc1;
    uint8 pwm1;
    uint8 ls[4];
} LED9532_MEM_TYPE;

typedef struct
{
	uint8 input_reg0;
	uint8 input_reg1;
	LED9532_MEM_TYPE rw_registers;
} LED9532_READ_MEM_TYPE;

typedef union
{
    struct
    {
        uint8 io_0  :1; //LSB
        uint8 io_1  :1;
        uint8 io_2  :1;
        uint8 io_3  :1;
        uint8 io_4  :1;
        uint8 io_5  :1;
        uint8 io_6  :1;
        uint8 io_7  :1;
        uint8 io_8  :1;
        uint8 io_9  :1;
        uint8 io_10 :1;
        uint8 io_11 :1;
        uint8 io_12 :1;
        uint8 io_13 :1;
        uint8 io_14 :1;
        uint8 io_15 :1; //MSB
    }bits;
    uint16 port_config;
} LED9532_IO_TYPE;

//=====================================================================================================================
//-------------------------------------- PUBLIC (Function Prototypes) -------------------------------------------------
//=====================================================================================================================
void Led9532__Initialize(void);
LED9532_STATUS_TYPE Led9532__Handler(void);
BOOL_TYPE Led9532__ForceWrite2Device(uint8 device );
BOOL_TYPE Led9532__CheckForcedWrite(uint8 device);
void Led9532__SetIntensity(uint8 device, LED9532_PWM_TYPE pwm, uint8 counts);
void Led9532__SetLed(uint8 device, uint8 led, LED9532_LED_TYPE state);
void Led9532__SetLedBuffer(LED9532_MEM_TYPE *Led_Buffer, uint8 device);
uint16  Led9532__ReadInputRegister(uint8 device);
void Led9532__SendReadCommand(uint8 device);
LED9532_READ_STATUS_TYPE Led9532__GetReadStatus(uint8 device);
LED9532_INPUT_TYPE Led9532__ReadInputPin(uint8 device, uint8 pin);
void Led9532__GetReadBuffer(LED9532_READ_MEM_TYPE *read_buffer ,uint8 device );

#endif // (LED_DRIVER_CAT9532 == ENABLED)
#endif // LED9532_H_
