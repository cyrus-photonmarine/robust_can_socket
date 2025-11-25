#ifndef CANSOCKET_H_
#define CANSOCKET_H_

#include <memory>
#include <string>
#include <cstdint>
#include <vector>

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
  struct Impl;
  std::unique_ptr<Impl> pimpl;
};

class Transmitter {
public:
  Transmitter(const std::string &interfaceName);
  ~Transmitter();
  void start();
  void stop();
  void send(const CanMessage &msg);

private:
  struct Impl;
  std::unique_ptr<Impl> pimpl;
};

} // namespace socketcan

#endif // CANSOCKET_H_
