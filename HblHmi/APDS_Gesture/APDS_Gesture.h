/*
 * Apds9960.h
 *
 *  Created on: Sep 26, 2019
 *      Author: GLNUG
 */

#ifndef SOURCE_XCATEGORY_HBL_HBLHMI_APDS9960_APDS9960_H_
#define SOURCE_XCATEGORY_HBL_HBLHMI_APDS9960_APDS9960_H_

//pre-processor directives for APDS Sensor initialization
#define APDSI2c_SLAVE_ADDRESS (0x39 << 1)
#define APDS_I2C_SPEED        I2C_400KHZ
#define APDS_I2C_ENUM_TYPE    I2C1
#define APDS_I2C_Address_Type I2C_ADDR_7BITS
#define Delay_counter_value   5000

typedef enum APDS_I2C_STATE_TYPE
{
    APDS_I2C_STATE_INIT = 0,
	APDS_I2C_STATE_WRITE,
	APDS_I2C_STATE_WAIT_END_WRITE,
	APDS_I2C_STATE_WAIT_END_WRITE_1,
	APDS_I2C_STATE_WAIT_END_WRITE_2,
	APDS_I2C_STATE_WAIT_END_WRITE_3,
	APDS_I2C_STATE_WAIT_END_WRITE_4,
	APDS_I2C_STATE_WAIT_END_WRITE_5,
	APDS_I2C_STATE_WAIT_END_WRITE_6,
	APDS_I2C_STATE_WAIT_END_WRITE_7,
	APDS_I2C_STATE_WAIT_END_WRITE_8,
	APDS_I2C_STATE_WAIT_END_WRITE_9,
	APDS_I2C_STATE_WAIT_END_WRITE_10,
	APDS_I2C_STATE_WRITE_VALUE,
	APDS_I2C_STATE_WRITE_ENTER_THRESHOLD,
	APDS_I2C_STATE_WRITE_EXIT_THRESHOLD,
	APDS_I2C_STATE_WRITE_ENTER_THRESHOLD_VALUE,
	APDS_I2C_STATE_WRITE_EXIT_THRESHOLD_VALUE,
	APDS_I2C_STATE_REQUEST_THRESHOLD_VALUE,
	APDS_I2C_STATE_REQUEST_EXIT_THRESHOLD_VALUE,
	APDS_I2C_STATE_CHECK_THRESHOLD_VALUE,
	APDS_I2C_STATE_CHECK_EXIT_THRESHOLD_VALUE,
	APDS_I2C_STATE_DELAY,
	APDS_I2C_STATE_REQUEST,
	APDS_I2C_STATE_REQUEST_GMODE,
	APDS_I2C_STATE_CHECK_READ,
	APDS_I2C_STATE_CHECK_READ_GMODE,
	APDS_I2C_STATE_READ,
	APDS_I2C_STATE_IDLE,
	APDS_I2C_GMODE_ENABLE,
	APDS_I2C_GMODE_ENABLE_CHECK,
	APDS_I2C_CHECK_GESTURE_AVAILABLE,
	APDS_I2C_STATE_GSTAT,
	APDS_I2C_STATE_REQUEST_GSTAT,
	APDS_I2C_STATE_CHECK_READ_GSTAT,
	APDS_I2C_STATE_CHECK_READ_GESTURE,
	APDS_I2C_STATE_READ_GMODE,
	APDS_I2C_STATE_READ_GESTURE,
	APDS_I2C_STATE_REQUEST_GFLVL,
	APDS_I2C_STATE_CHECK_READ_GFLVL,
	APDS_I2C_STATE_GFIFO_U,
	APDS_I2C_STATE_REQUEST_GFIFO_U,
	APDS_I2C_STATE_CHECK_READ_GFIFO_U

} APDS_I2C_STATE_TYPE;

/* APDS-9960 register addresses */
#define APDS9960_ENABLE         0x80
#define APDS9960_ATIME          0x81
#define APDS9960_WTIME          0x83
#define APDS9960_AILTL          0x84
#define APDS9960_AILTH          0x85
#define APDS9960_AIHTL          0x86
#define APDS9960_AIHTH          0x87
#define APDS9960_PILT           0x89
#define APDS9960_PIHT           0x8B
#define APDS9960_PERS           0x8C
#define APDS9960_CONFIG1        0x8D
#define APDS9960_PPULSE         0x8E
#define APDS9960_CONTROL        0x8F
#define APDS9960_CONFIG2        0x90
#define APDS9960_ID             0x92
#define APDS9960_STATUS         0x93
#define APDS9960_CDATAL         0x94
#define APDS9960_CDATAH         0x95
#define APDS9960_RDATAL         0x96
#define APDS9960_RDATAH         0x97
#define APDS9960_GDATAL         0x98
#define APDS9960_GDATAH         0x99
#define APDS9960_BDATAL         0x9A
#define APDS9960_BDATAH         0x9B
#define APDS9960_PDATA          0x9C
#define APDS9960_POFFSET_UR     0x9D
#define APDS9960_POFFSET_DL     0x9E
#define APDS9960_CONFIG3        0x9F
#define APDS9960_GPENTH         0xA0
#define APDS9960_GEXTH          0xA1
#define APDS9960_GCONF1         0xA2
#define APDS9960_GCONF2         0xA3
#define APDS9960_GOFFSET_U      0xA4
#define APDS9960_GOFFSET_D      0xA5
#define APDS9960_GOFFSET_L      0xA7
#define APDS9960_GOFFSET_R      0xA9
#define APDS9960_GPULSE         0xA6
#define APDS9960_GCONF3         0xAA
#define APDS9960_GCONF4         0xAB
#define APDS9960_GFLVL          0xAE
#define APDS9960_GSTATUS        0xAF
#define APDS9960_IFORCE         0xE4
#define APDS9960_PICLEAR        0xE5
#define APDS9960_CICLEAR        0xE6
#define APDS9960_AICLEAR        0xE7
#define APDS9960_GFIFO_U        0xFC //consecutive memory locations where 8 bit results corresponding to each diode is present
#define APDS9960_GFIFO_D        0xFD
#define APDS9960_GFIFO_L        0xFE
#define APDS9960_GFIFO_R        0xFF

//The ENABLE register is used to power the device on/off, enable functions and interrupts.
/* Bit fields */
#define APDS9960_PON            0b00000001
#define APDS9960_AEN            0b00000010
#define APDS9960_PEN            0b00000100
#define APDS9960_WEN            0b00001000
#define APDS9960_AIEN           0b00010000
#define APDS9960_PIEN           0b00100000
#define APDS9960_GEN            0b01000000  /*Gesture Enable. When asserted, the gesture state machine can be activated. Activation is subject to the states of PEN and GMODE bits.*/
#define APDS9960_GVALID         0b00000001  //will be set when if the FIFO level(GFLVL) becomes greater or equal to the threshold value(GFIFOTH) indicating valid data is available

#define APDS9960_GVALID_1         0x01
#define APDS9960_VALID_CHECK      0x41

//Enum for getting direction of gesture
typedef enum APDS_I2C_DIR_TYPE {
  DIR_NONE,
  DIR_LEFT,
  DIR_RIGHT,
  DIR_UP,
  DIR_DOWN,
  DIR_NEAR,
  DIR_FAR,
  DIR_ALL
}APDS_I2C_DIR_TYPE;

void Apds__Initialize(void);
void Apds__Handler(void);

#endif /* SOURCE_XCATEGORY_HBL_HBLHMI_APDS9960_APDS9960_H_ */
