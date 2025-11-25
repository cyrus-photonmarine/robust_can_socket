#include "CANSocket.h"
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <thread>
#include <unistd.h>

namespace socketcan {

CanMessage::CanMessage() : id(0), dlc(0), data{0}, timestamp_us(0) {}
CanMessage::CanMessage(uint32_t msg_id, const std::vector<uint8_t> &vec_data,
                       uint64_t ts)
    : id(msg_id), dlc(vec_data.size()), timestamp_us(ts) {
  std::fill(data, data + 8, 0);
  std::copy(vec_data.begin(),
            vec_data.begin() + std::min<std::size_t>(8, vec_data.size()), data);
}

std::vector<uint8_t> CanMessage::toVector() const {
  return std::vector<uint8_t>(data, data + dlc);
}

void CanMessage::print() const {
  std::cout << "[MSG] ID: 0x" << std::hex << id << "  Data:";
  for (int i = 0; i < dlc; ++i)
    std::cout << " " << std::hex << static_cast<int>(data[i]);
  std::cout << std::dec << std::endl;
}

class CANSocket::Impl {
public:
  Impl(const std::string &interfaceName)
      : m_interfaceName(interfaceName), m_socket(-1) {}
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

CANSocket::CANSocket(const std::string &interfaceName)
    : pimpl(std::make_unique<Impl>(interfaceName)) {}
CANSocket::~CANSocket() { pimpl->close(); }
bool CANSocket::initialize() { return pimpl->initialize(); }
void CANSocket::close() { pimpl->close(); }
bool CANSocket::sendMessage(uint32_t id, const std::vector<uint8_t> &data) {
  return pimpl->sendMessage(id, data);
}
bool CANSocket::receiveMessage(uint32_t &id, std::vector<uint8_t> &data,
                               uint64_t &timestamp) {
  return pimpl->receiveMessage(id, data, timestamp);
}

bool CANSocket::Impl::initialize() {
  std::lock_guard<std::mutex> lock(m_socketMutex);
  m_isSocketValid = false;
  const int restart_ms = 100;
  m_socket = -1;

  // One Socket to rule them all. One loop to find them.
  do {
    m_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (m_socket < 0) {
      perror("socket");
      std::cerr << "[CANSocket] Waiting for CAN interface: " << m_interfaceName
                << " to become available (socket failed)" << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } else {
      setsockopt(m_socket, SOL_CAN_RAW, 6, &restart_ms, sizeof(restart_ms));
      std::strncpy(m_ifr.ifr_name, m_interfaceName.c_str(), IFNAMSIZ - 1);
      if (ioctl(m_socket, SIOCGIFINDEX, &m_ifr) < 0) {
        perror("ioctl");
        std::cerr << "[CANSocket] Waiting for CAN interface: "
                  << m_interfaceName << " to become available (ioctl failed)"
                  << std::endl;
        ::close(m_socket);
        m_socket = -1;
      }
    }
  } while (m_socket < 0);

  // One Socket to bring them all
  std::memset(&m_addr, 0, sizeof(m_addr));
  m_addr.can_family = AF_CAN;
  m_addr.can_ifindex = m_ifr.ifr_ifindex;

  // and in the darkness bind them.
  if (bind(m_socket, (struct sockaddr *)&m_addr, sizeof(m_addr)) < 0) {
    perror("bind");
    ::close(m_socket);
    m_socket = -1;
    return false;
  }

  int flags = fcntl(m_socket, F_GETFL, 0);
  if (flags < 0 || fcntl(m_socket, F_SETFL, flags | O_NONBLOCK) < 0) {
    perror("fcntl");
    ::close(m_socket);
    m_socket = -1;
    return false;
  }

  std::cout << "[CANSocket] CAN channel initialized successfully on interface: "
            << m_interfaceName << std::endl;
  m_isSocketValid = true;

  return true;
}

void CANSocket::Impl::close() {
  std::lock_guard<std::mutex> lock(m_socketMutex);

  if (m_socket >= 0) {
    ::close(m_socket);
    m_socket = -1;
    std::cout << "CAN channel closed successfully." << std::endl;
  }
  m_isSocketValid = false;
}

bool CANSocket::Impl::sendMessage(uint32_t id,
                                  const std::vector<uint8_t> &data) {
  if (!m_isSocketValid) {
    std::cerr << "[CANSocket] Socket not valid, skipping send." << std::endl;
    return false;
  }
  bool need_reinit = false;
  {
    std::lock_guard<std::mutex> lock(m_socketMutex);

    if (m_socket < 0) {
      std::cerr << "[CANSocket] Invalid socket for send." << std::endl;
      return false;
    }

    if (data.size() > 8) {
      std::cerr << "Data too long" << std::endl;
      return false;
    }

    struct can_frame frame = {};
    frame.can_id = id;
    frame.can_dlc = data.size();
    std::copy(data.begin(), data.end(), frame.data);

    ssize_t bytes_written = write(m_socket, &frame, sizeof(frame));
    if (bytes_written == sizeof(frame)) {
      return true;
    } else {
      perror("CANSocket write failed");
      need_reinit = true;
    }
  }

  // Reinit outside lock
  if (need_reinit) {
    std::cerr << "[CANSocket] Reinitializing socket outside lock..."
              << std::endl;
    close();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    return initialize(); // optional: retry send if you want
  }

  return false;
}

bool CANSocket::Impl::receiveMessage(uint32_t &id, std::vector<uint8_t> &data,
                                     uint64_t &timestamp) {
  std::lock_guard<std::mutex> lock(m_socketMutex);

  if (!m_isSocketValid || m_socket < 0) {
    errno = ENOTCONN;
    return false;
  }

  struct can_frame frame;
  ssize_t nbytes = read(m_socket, &frame, sizeof(frame));

  if (nbytes < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK)
      return false;
    return false;
  }

  id = frame.can_id;
  data.assign(frame.data, frame.data + frame.can_dlc);
  timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::system_clock::now().time_since_epoch())
                  .count();

  return true;
}

Transmitter::Transmitter(const std::string &interfaceName)
    : socketcan::CANSocket(interfaceName), m_running(false) {}
Transmitter::~Transmitter() {
  m_running = false;
  close();
  if (m_handle.joinable()) {
    m_handle.join();
  }
}
void Transmitter::start() {
  if (!m_running) {
    initialize();
    m_running = true;
    m_handle = std::thread(&Transmitter::runloop, this);
  }
}

void Transmitter::stop() {
  if (m_running) {
    m_running = false;
    if (m_handle.joinable()) {
      m_handle.join();
    }
  }
}

void Transmitter::send(const socketcan::CanMessage &msg) {
  m_queue.push(std::move(msg));
}
void Transmitter::runloop() {
  CanMessage msg;
  while (m_running) {
    std::optional<CanMessage> m = m_queue.pop();
    if (m.has_value()) {
      msg = *m;
      msg.print();
      if (!sendMessage(msg.id, msg.toVector())) {
        std::cerr << "[SEND] Failed to send ID: 0x" << std::hex << msg.id
                  << std::dec << std::endl;
      }
    }
  }
}
} // namespace socketcan
