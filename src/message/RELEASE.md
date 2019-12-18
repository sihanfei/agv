# Release 1.0

#### 20190508
common_msgs/Point.msg                  修改为 common_msgs/Point2D.msg  
common_msgs/PathData.msg               修改为 common_msgs/PathPoint.msg  
common_msgs/REFPoint.msg               更新内容  
common_msgs/TargetPoint.msg            更新内容  
common_msgs/UltrasonicPoint.msg        新建文件  

control_msgs/com2veh.msg               更新内容  
control_msgs/ADControlAGV.msg          暂不使用  
control_msgs/ADControlCar.msg          暂不使用  
control_msgs/AGVStatus.msg             暂不使用  
control_msgs/CarStatus.msg             暂不使用  

integrated_nav_msgs/IMUAndGNSSInfo.msg 修改为 location_sensor_msgs/IMUAndGNSSInfo.msg  

fusion_data_msgs/fusion_data_msgs.msg  修改为 location_sensor_msgs/fusion_data_msgs.msg  
fusion_data_msgs/LidarInfo.msg         修改为 location_sensor_msgs/LidarInfo.msg  
fusion_data_msgs/UWBInfo.msg           修改为 location_sensor_msgs/UWBInfo.msg  

location_msgs/LocationPoseInfo.msg     修改为 location_msgs/FusionDataInfo.msg  
location_msgs/IMUAndGNSSInfo.msg       删除文件  

perception_sensor_msgs/ObjectInfo.msg     修改为 common_msgs/DetectionInfo  
perception_sensor_msgs/ObjectList.msg     更新内容  
perception_sensor_msgs/UltrasonicInfo.msg 新建文件  

map_msgs/REFPointsInfo.msg             修改为 map_msgs/REFPointArray.msg  
map_msgs/RefPoint.msg                  删除文件  

hmi_msgs/HMIControlAD.msg              新建文件  
hmi_msgs/ADStatus.msg                  新建文件  
hmi_msgs/VMSControlAD.msg              新建文件  

#### 20190611 by--lcl
1、在hmi_msgs中增加FsmControVcuDriver.msg，用于FSM调度VCU控制驱动的控制权，具体内容见《软件消息类型说明》9.3 状态机控制命令FsmControVcuDriver.msg.  
2、在hmi_msgs的VMSControlAD.msg中添加fsm_control和message_num字段，并修改“信息编码”字段名称，具体内容见《软件消息类型说明》9.1 控制命令 VMSControlAD.msg.  
