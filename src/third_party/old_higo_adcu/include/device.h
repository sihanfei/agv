#ifndef __DEVICE_H__
#define __DEVICE_H__
#include <stdint.h>

/** 
* @brief adcu support devices.
*/
typedef enum
{
	adcuCAN = 0x80,			/*!< adcu can device */
	adcuRS232,				/*!< adcu rs232 device */
	adcuRS485,				/*!< adcu rs485 device */
	adcuLIN,				/*!< adcu LIN device */
	adcuADC,				/*!< adcu ADC device */
	adcuDAC,				/*!< adcu DAC device */
	adcuHIGHSIDE,			/*!< adcu HighSide device */
	adcuFLEXRAY,			/*!< adcu FLEXRAY device */
	adcuPWM_OUT,			/*!< adcu PWM_OUT device */
	adcuPWM_IN,				/*!< adcu PWM_IN device */
	adcuSERVER,
	adcuPOWER,
}adcuDeviceType;


/** 
* @brief adcu device channels.
*/
typedef enum 
{
	ADCU_CHANNEL_1 = 1,     /*!< adcu channel 1 */
	ADCU_CHANNEL_2,         /*!< adcu channel 2 */
	ADCU_CHANNEL_3,         /*!< adcu channel 3 */
	ADCU_CHANNEL_4,         /*!< adcu channel 4 */
	ADCU_CHANNEL_5,         /*!< adcu channel 5 */
	ADCU_CHANNEL_6,         /*!< adcu channel 6 */
	ADCU_CHANNEL_7,         /*!< adcu channel 7 */
	ADCU_CHANNEL_8,         /*!< adcu channel 8 */
	ADCU_CHANNEL_9,         /*!< adcu channel 9 */
	ADCU_CHANNEL_10,        /*!< adcu channel 10*/
	ADCU_CHANNEL_MAX,
}adcuDeviceChannel;



/** 
* @brief can data max size.
*/
#define CAN_DATA_SIZE 8

/** 
* @brief adcu can data struct.
*/
#pragma pack(1)
typedef struct
{
	uint8_t ide;                            /*!< 0: standard frame, 1: extended frame */
	uint8_t dlc;                            /*!< data length */
	uint8_t rtr;                            /*!< remote transmission request */
	uint8_t prio;                           /*!< priority */
	uint32_t id;                            /*!< identify */
	uint8_t can_data[CAN_DATA_SIZE];        /*!< datas */
}*p_adcuCanData, adcuCanData;
#pragma pack(0)

/** 
* @brief rs232 data max size.
*/
#define RS232_DATA_SIZE 16

/** 
* @brief adcu rs232 data struct.
*/
#pragma pack(1)
typedef struct
{
	uint8_t data[RS232_DATA_SIZE];            /*!< data */
}*p_adcuRS232Data, adcuRS232Data;

#pragma pack(0)

/** 
* @brief adcu rs232/rs485 option struct.
*/
#pragma pack(1)
typedef struct
{
	uint32_t baudrate;            				/*!< baudrate */
	uint8_t wordlength;							/*!< bit length of a frame. 0:7 bits , 1:8bits*/
	uint8_t parity;								/*!< parity 0:even,1:odd,2:fill with 0,3:fill with 1,4:none*/
	uint8_t stopbit;							/*!< stop bit 0:1 stop bit,1:2 stop bits,2:3 stop bits,3:reserved*/
}*p_adcuUartOpt, adcuUartOpt;

#pragma pack(0)

/** 
* @brief adcu pwm data struct.
*/
#pragma pack(1)
typedef struct
{
	uint16_t frequency;                          /*!< 0~65535 pwm out frequency*/
	uint8_t dutyCycle;                           /*!< duty Cycle*/
}*p_adcuPwmData, adcuPwmData;
#pragma pack(0)


/** 
* @brief adcu DAC data struct.
*/
#pragma pack(1)
typedef struct
{
    uint16_t voltage_mv;                          /*!< voltage value(mV)*/
}*p_adcuDacData, adcuDacData;
#pragma pack(0)

/**
* @brief Lin data max size.
*/
#define LIN_DATA_SIZE 8


#pragma pack(1)
/**
* @brief adcu Lin data struct.
*/
typedef struct
{
	uint8_t len;                            /*!< data length */
	uint8_t dir;                            /*!< 0:receive 1:send */
	uint8_t ccs;                            /*!< check type */
	uint8_t id;                             /*!< identify */
	uint8_t data[LIN_DATA_SIZE];        	/*!< datas */
}*p_adcuLinData, adcuLinData;

#pragma pack(0)


#pragma pack(1)
/**
* @brief adcu HighSide data struct.
*/
typedef struct
{
    uint8_t state;                            /*!< data 0:disable,1:enable*/
}*p_adcuHSData, adcuHSData;

#pragma pack(0)

#endif
