/**
  ******************************************************************************
  * @file    *.h
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
#ifndef _ARSx08_H
#define _ARSx08_H


#define Valid 	1
#define Invalid 0

#define SendObj 1
#define SendTar 2


#define RANGE  250      /*¼ì²â¾àÀë*/
#define ELEVA  15     /*¸©Ñö½Ç  0¡ã->0.25¡ã->0.50¡ã 32¡ã   Sky->Road */


/*Radar Configuration message structure (0x200) (Bytes 5-7 and Bits in White are unused)*/
/*
If the validity bit is set to Valid/True, the
corresponding parameter will be updated in the ARS, otherwise it is ignored. If a
parameter is updated, it will also be stored in non-volatile memory so that it is
automatically set at startup on any subsequent power up.
*/
struct MsgRadarConfig
{
    //Valid-->1 or Invalid-->0
    uint8_t radar_range_lengthV: 1;
    uint8_t radar_elevationV: 1;
    uint8_t radar_enablePowerReductionV: 1;
    uint8_t radar_output_typeV: 1;
    uint8_t radar_sensor_idV: 1;
    uint8_t : 3; //unused

    uint8_t radar_range_length: 8;
    uint8_t radar_elevation: 8;
    uint8_t radar_enablePowerReduction: 1;
    uint8_t radar_output_type: 2;
    uint8_t : 5; //unused
    uint8_t radar_sensor_id: 3;
    uint8_t : 5; //unused

    uint8_t : 8; //unused
    uint8_t : 8; //unused
    uint8_t : 8; //unused
};


//-----------------------------------------------------------------------
/*  Radar State message description (0x201) */
struct MsgRadarState
{
    uint8_t : 1; //unused
    uint8_t CurrentRadarPower: 1;
    uint8_t : 1; //unused
    uint8_t SensTempErr: 1;
    uint8_t SensBlockage: 1;
    uint8_t SensDef: 1;
    uint8_t SupVolt_L: 1;
    uint8_t RadarPowerReduction: 1; /*Power reduction mode*/

    uint8_t SensorID: 4;            /*Sensor ID*/
    uint8_t : 4; //unused
    uint8_t RxInvalid: 2;
    uint8_t NVMreadStatus: 2;
    uint8_t NVMwriteStatus: 1;
    uint8_t : 3; //unused
    uint8_t currElevationCal: 8;    /*Elevation of the sensor's plate - 0.25deg*/
    uint8_t currRangeLengthCal: 8;  /*Radar range length */
    uint8_t swMajorVersion: 8;
    uint8_t swMinorVersion: 8;
    uint8_t	swBuildVersion: 8;
};
/*CAN1_Target_Status message description (0x600)*/
struct MsgTargetStatus
{
    uint8_t NoOfTargetsNear;
    uint8_t NoOfTargetsFar;
};
/* CAN1_Obj_Status message structure (0x60A) (Byte 7 is not used)*/
struct MsgObjectStatus
{
    uint8_t NoOfObjects;
    uint8_t MeasCounter;

};
/**/

struct ObjList
{
    uint8_t Obj_ID;//0->63

    uint8_t Obj_Length;
    uint8_t Obj_Width;
    uint8_t Obj_ProbOfExist;/*Probability of existence calculated for an Object*/

    uint8_t Obj_DynProp;/*Dynamic property*/
    uint8_t Obj_MessStat;/*Object measurement status*/

    float   Obj_LongDispl;//0->204m ,res 0.1m
    float   Obj_LatDispl;//-51.9m->52m, res 0.1m
    
    float   Obj_VerlLong;//Relative velocity in longitudinal direction (x)  -  res 0.0625m/s ,offset -128m/s
    float   Obj_VerlLat; //Relative velocity in lateral direction (y)
    float   Obj_AccelLong;//Relative longitudinal acceleration  -res 0.0625m/s^2 ,offset -16m/s^2

};

/***************************/
void ARS_Init();
struct MsgRadarState GetRadarState(CanRxMsg *canrxMsg);
struct ObjList ARS_Obj_Handle(CanRxMsg *CANRxMsg);



#endif