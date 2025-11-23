#ifndef CANSOCKET_H_
#define CANSOCKET_H_

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <string>
#include <vector>
#include <mutex>
#include <cstdint>
#include <iostream>
#include <atomic>


#define BodyCAN             "can3"
#define SensorCAN           "can1"
#define Steering_Motor_CAN  "can2"

class CANSocket {
public:
    CANSocket(const std::string& interfaceName);
    ~CANSocket();

    bool initialize();
    void close();
    bool sendMessage(uint32_t id, const std::vector<uint8_t>& data);
    bool receiveMessage(uint32_t& id, std::vector<uint8_t>& data, uint64_t& timestamp);


private:
    std::string m_interfaceName;  
    int m_socket;                 
    struct sockaddr_can m_addr;   
    struct ifreq m_ifr;           
    std::mutex m_socketMutex;
    int m_rxFailureCount;
    int m_txFailureCount;
    std::atomic<bool> m_isSocketValid{true}; 
};

#endif // CANSOCKET_H_
