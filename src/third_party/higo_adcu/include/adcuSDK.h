#ifndef __ADCU_SDK_H__
#define __ADCU_SDK_H__

#include "device.h"

/** 
* @brief device Device operating status.
*/
typedef enum{
    ADCU_DEV_STATUS_NORMAL=0,	/*!< Device operating status is ok */
    ADCU_DEV_STATUS_ABNORMAL,	/*!< Device operating status is error */
}ADCU_DEV_STATUS;

/**
* @brief Init running state 
* @retval 0 : success
* @retval -1 : if an error occurred
*/
int adcuSDKInit();

/**
* @brief open specify device with channel,and if open successfully return a positive 
	integer represents device id else return 0.
* @param adcuDeviceType : adcu support devices,eg.adcuCAN/adcuRS232.
* @param channel : channel num, you can use adcuChannel:ADCU_CHANNEL_1, ADCU_CHANNEL_2 ,
                        ADCU_CHANNEL_3, ADCU_CHANNEL_4.
* @retval <0 : if an error occurred
* @retval DevId : a positive integer  
*/
int adcuDevOpen(adcuDeviceType DevType ,int channel);

/**
* @brief Read 'size' bytes into BUF from 'DevId'.  Return the
   number read, -1 for errors.
* @param DevId : function adcuDevOpen() return value.
* @param  buf : a point of buffer to receive data.
* @param  size :  receive data size.
* @retval <=0 : read error.
* @retval >0 : the actual size of read. 
*/
int adcuDevRead(int DevId, uint8_t *buf,int size);

/**
* @brief Write 'size' bytes of BUF to 'DevId'.  Return the number written, or -1.
* @param DevId : function adcuDevOpen() return value.
* @param buf : a point of buffer to write data.
* @param size : write size.
* @retval <=0 : write error.
* @retval >0 : the actual size of write. 
*/
int adcuDevWrite(int DevId, uint8_t *buf,int size);

/**
* @brief get the status of device.
* @param DevId : function adcuDevOpen() return value.
* @retval ADCU_DEV_STATUS_NORMAL 
* @retval ADCU_DEV_STATUS_ABNORMAL
*/
int adcuDevStatus(int DevId);

/**
* @brief close device channel
* @param DevId : function adcuDevOpen() return value.
* @retval ADCU_DEV_STATUS_NORMAL 
* @retval ADCU_DEV_STATUS_ABNORMAL
*/
int adcuDevClose(int DevId);

/**
* @brief clear running state
*/
void adcuSDKDeinit();

#endif
