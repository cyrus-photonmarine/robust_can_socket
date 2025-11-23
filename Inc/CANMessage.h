#ifndef SOCKET_CAN_MSG_H_
#define SOCKET_CAN_MSG_H_

#include <cstdint>
#include <vector>

struct CanMessage {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];  
    uint64_t timestamp_us;  
    CanMessage();
    CanMessage(uint32_t msg_id, const std::vector<uint8_t>& vec_data, uint64_t ts = 0);
    std::vector<uint8_t> toVector() const; 
};

#endif // SOCKET_CAN_MSG_H_
