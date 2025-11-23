#ifndef CANSOCKET_H_
#define CANSOCKET_H_

#include <atomic>
#include <cstdint>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <mutex>
#include <net/if.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <vector>

#define BodyCAN "can3"
#define SensorCAN "can1"
#define Steering_Motor_CAN "can2"

namespace socketcan {
struct CanMessage {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
  uint64_t timestamp_us;
  CanMessage();
  CanMessage(uint32_t msg_id, const std::vector<uint8_t> &vec_data,
             uint64_t ts = 0);
  std::vector<uint8_t> toVector() const;
  void print() const;
};

class CANSocket {
public:
  CANSocket(const std::string &interfaceName);
  ~CANSocket();

  bool initialize();
  void close();
  bool sendMessage(uint32_t id, const std::vector<uint8_t> &data);
  bool receiveMessage(uint32_t &id, std::vector<uint8_t> &data,
                      uint64_t &timestamp);

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
} // namespace socketcan

#endif // CANSOCKET_H_
