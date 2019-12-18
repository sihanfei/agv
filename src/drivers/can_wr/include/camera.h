#ifndef DRIVERS_CAN_WR_INCLUDE_CAMERA_H_
#define DRIVERS_CAN_WR_INCLUDE_CAMERA_H_

#include "can_rw.h"
#include "driver_monitor.h"
#include "logdata.h"
#include "relay.h"

namespace superg_agv
{
namespace drivers
{
class CAMERA : private CANDrivers,public Driver_Monitor
{
    typedef void (CAMERA::*pFun)(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);

    map< uint32_t, pFun > m_pHandlerMap;

    enum camera_recv_enum
    {
        DISTINGUISH,
        MAT,
        NONE,
    };

    enum Camera_State
    {
        Allstart_Camera,
        Getstatus_Camera,
        Waitstatus_Camera,
        Work_Camera,
        Error_Camera
    };

    enum Camera_Task
    {
        ALL_StartWork,
        ALL_GetStatus,
        ALL_StopWork,
        Object_StopWork,
        Object_StartWork,
        Object_GetWork,
        Wait_AllStatus,
        Wait_ObjectStatus
    };

    private:
        int devid;
        std::mutex mtx_camera;
        std::mutex mtx_camera_matData1;
        std::mutex mtx_camera_matData2;
        std::mutex mtx_camera_ObjectData1;
        std::mutex mtx_camera_ObjectData2;
        std::mutex mtx_camera_LaneData1;
        std::mutex mtx_camera_LaneData2;
        std::condition_variable cond_camera;

        list<Camera_Task> list_camera_wirte_task;
        list<Camera_Task> list_camera_read_task;

        camera_ObjectData_state array_camera_ObjectData[8];
        camera_LaneData_state array_camera_LaneData[8];

        camera_ObjectData_state cameraObjectData_ready_1;
        camera_ObjectData_state cameraObjectData_ready_2;

        cameraObjectInfo cameraObjectInfo_1;
        cameraObjectInfo cameraObjectInfo_2;

        camera_LaneData_state cameraLaneData_ready_1;
        camera_LaneData_state cameraLaneData_ready_2;

        cameraLaneInfo cameraLaneInfo_1;
        cameraLaneInfo cameraLaneInfo_2;


        matDataStr matData_1;
        matDataStr matData_2;
        matDataStr matData_ready_1;
        matDataStr matData_ready_2;

        bool bCameraInit;
        int camerastatus[9];
    public:
        CAMERA();
        ~CAMERA();

        void CANCamera(int &dev_, int &ch_);
        void camera_Sender();
        void CANCameraWrite(int &dev_, int &ch_);

        void sensorDataParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void sensorControl(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);

        void objectHeadParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void objectDataFirstParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void objectDataSecondParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void laneHeadParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void laneDataFirstParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void laneDataSecondParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void cameraWorkStatusParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);

        void cameraAllStartWork(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void cameraAllGetStatus(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void cameraAllStopAcquire(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void cameraStartWork(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void cameraStopWork(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);

        void cameraGetStatus(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);

        void matHeadParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void matRawParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void matDataParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
        void matTailParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype);
    protected:
};
}
}

#endif
