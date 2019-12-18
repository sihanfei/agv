/**
  ******************************************************************************
  * @file    *.c
  * @author  pengli.wang
  * @version V3.5.0
  * @date    08-April-2018
  * @brief   Handle the radar output data .
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, HURYS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2018 HURYS</center></h2>
  ******************************************************************************
  */
#include "ARSx08.h"

union CANTxMsgData
{

    uint8_t Data[8];
    struct MsgRadarConfig MsgRadarConfigs;

};

union CANRadarState
{

    uint8_t Data[8];
    struct MsgRadarState MsgRadarStates;

};

/*
Init ARS Radar Configration when the program load.
noticed that the sensor's ID must be 0.
*/
void ARS_Init()
{
    uint8_t i = 0;
    CanTxMsg radarConfigMsg;
    union CANTxMsgData ARS_Initstruct;


    radarConfigMsg.StdId = 0x0200;
    radarConfigMsg.ExtId = 0x0000;
    radarConfigMsg.DLC   = 0x08;
    radarConfigMsg.IDE = CAN_ID_STD;
    radarConfigMsg.RTR = CAN_RTR_DATA;

    radarConfigMsg.Data[0]=0xC8;
    radarConfigMsg.Data[1]=0x00;
    radarConfigMsg.Data[2]=0x00;
    radarConfigMsg.Data[3]=0x00;
    radarConfigMsg.Data[4]=0x08;
    radarConfigMsg.Data[5]=0x90;
    radarConfigMsg.Data[6]=0x00;
    radarConfigMsg.Data[7]=0x00;    
    
    /*Transmit the can config command*/
    CAN_Transmit(CAN1,&radarConfigMsg);
}

/*½âÎö0x0201Êý¾Ý*/
struct MsgRadarState GetRadarState(CanRxMsg *canrxMsg)
{
    uint8_t i = 0;
    union CANRadarState canradarstate;
    for(; i < 8; i++)
    {
        canradarstate.Data[i] = canrxMsg->Data[i];
    }
    return canradarstate.MsgRadarStates;
}
struct ObjList ARS_Obj_Handle(CanRxMsg *CANRxMsg)
{

#define ARS408

	struct ObjList  CAN1_Obj_1_Msg;

#ifdef ARS408
    CAN1_Obj_1_Msg.Obj_ID           = (CANRxMsg->Data[0]>>0);
    CAN1_Obj_1_Msg.Obj_LongDispl    = ((CANRxMsg->Data[1]<<5)|(CANRxMsg->Data[2]>>3))*0.2 - 500;
    CAN1_Obj_1_Msg.Obj_LatDispl     = ((((CANRxMsg->Data[2]&0x07)<<8)|(CANRxMsg->Data[3]>>0))*0.2)-204.6;
    CAN1_Obj_1_Msg.Obj_DynProp      = (CANRxMsg->Data[6]>>0)&0x07;
    CAN1_Obj_1_Msg.Obj_VerlLong     = (((CANRxMsg->Data[4]&0xFF)<<2)|(CANRxMsg->Data[5]>>6))*0.25 - 128;
    CAN1_Obj_1_Msg.Obj_VerlLat      = (((CANRxMsg->Data[5]&0x3F)<<3)| ((CANRxMsg->Data[6]&0xE0)>>5))*0.25-64;
#endif    
    return CAN1_Obj_1_Msg;
}























