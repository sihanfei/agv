# cannet udp packet.

time stamp              # packet timestamp
uint8 ide         #< 0: standard frame, 1: extended frame > 1为扩展帧,0为标准帧
uint8 rtr         #< remote transmission request >1为远程帧,0为数据帧
uint32 can_id
uint8  can_data_len
uint8[8] can_data        # packet contents