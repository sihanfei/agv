Header header         # standard ROS message header
uint8 iflag
uint8 Object_NofObjects   # Number of objects (max. 100 Objects)
uint16 Object_MeasCounter # Measurement cycle counter (counting up since startup of sensor and restarting at 0 when > 65535)
uint32 obj60b_count
uint32 obj60c_count
ContiArs60bData[] ars60Bd_array
ContiArs60cData[] ars60C_array