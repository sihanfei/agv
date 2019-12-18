Header header
uint8 fsm_control   # 0 1 2 3
uint8 CMD_TYpe	   # receiver为3时： 0– 空  6–紧停 7– 复位 8– 任务暂停 9– 任务继续  10-任务取消 12– 滑停
uint8 receiver 	   # 0- 空 1- FSM 2- Planning 3- Exception 4- Task 5-Others
uint8 message_num   # 信息编码数量
string[] message_code  # 信息编码