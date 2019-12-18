#ifndef CANBUS_CONSTS_H_
#define CANBUS_CONSTS_H_
#include <cstdint>

const int32_t CAN_FRAME_SIZE = 8;
const int32_t MAX_CAN_SEND_FRAME_LEN = 1;
const int32_t MAX_CAN_RECV_FRAME_LEN = 10;

const int32_t CANBUS_MESSAGE_LENGTH = 8;  // according to ISO-11891-1
const int32_t MAX_CAN_PORT = 3;

#endif  // MODULES_DRIVERS_CANBUS_COMMON_CANBUS_CONSTS_H_
