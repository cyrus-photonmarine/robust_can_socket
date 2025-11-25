#ifndef CANSOCKET_H_
#define CANSOCKET_H_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <mutex>
#include <net/if.h>
#include <optional>
#include <queue>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <thread>
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
  std::atomic<bool> m_isSocketValid{true};
};

template <typename T> class ThreadSafeQueue {
private:
  std::queue<T> m_queue;
  mutable std::mutex m_mutex;
  std::condition_variable m_queue_empty;

public:
  void push(const T &val) {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_queue.push(std::move(val));
    lock.unlock();
    m_queue_empty.notify_one();
  }

  std::optional<T> pop() {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_queue_empty.wait_for(lock, std::chrono::milliseconds(100),
                           [this] { return !m_queue.empty(); });
    if (m_queue.empty())
      return std::nullopt;
    T value = std::move(m_queue.front());
    m_queue.pop();
    return value;
  }
};

class Transmitter : public CANSocket {
public:
  Transmitter(const std::string &interfaceName);
  ~Transmitter();
  void start();
  void stop();
  void send(const CanMessage &msg);

private:
  ThreadSafeQueue<CanMessage> m_queue;
  std::atomic<bool> m_running;
  std::thread m_handle;
  void runloop();
};

} // namespace socketcan

#endif // CANSOCKET_H_
