Header header
# FSM
uint8 Agv_Status
uint8 Node_Status
uint16 Node_Error_Num
string[] Node_Error_Code
uint8 Hardware_Status
uint16 Hardware_Error_Num
string[] Hardware_Error_Code
uint8 planning_task_ID
uint8 planning_task_status
uint8 operation_task_ID
uint8 operation_task_status
uint8 exception_task_ID
uint8 exception_task_status
bool ad_enbale_status

# exception
float32 ActualSpd		# m/s  VCU反馈车体实际速度
bool EStopStatus		# 紧停状态

