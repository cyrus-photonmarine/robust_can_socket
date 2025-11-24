#include "CANSocket.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

constexpr int TX_MSGS_LEN = 3;
std::array<int, TX_MSGS_LEN> sampleTimes = {10, 20, 100}; // ms

std::atomic<bool> running{true};

std::array<uint32_t, 8> RX_IDS = {0x101, 0x102, 0x103, 0x201,
                                  0x202, 0x203, 0x301, 0x302};

std::mutex tx_queue_mutex;
std::queue<socketcan::CanMessage> tx_queue;
std::condition_variable tx_queue_empty;

void tx_queue_message_push(const socketcan::CanMessage &msg) {
  std::cout << "[SEND] Enqueueing ID: 0x" << std::hex << msg.id << std::dec
            << std::endl;
  std::unique_lock<std::mutex> lock(tx_queue_mutex);
  tx_queue.push(msg);
  lock.unlock();
  tx_queue_empty.notify_one();
}

void sendLoop(socketcan::CANSocket *socket) {
  while (running) {
    std::cout << "[SEND] Waiting..." << std::endl;
    std::unique_lock<std::mutex> lock(tx_queue_mutex);
    tx_queue_empty.wait(lock, [] { return !tx_queue.empty(); });
    do {
      std::cout << "[SEND] Sending..." << tx_queue.size() << std::endl;
      socketcan::CanMessage msg = tx_queue.front();
      if (!socket->sendMessage(msg.id, msg.toVector())) {
        std::cerr << "[SEND] Failed to send ID: 0x" << std::hex << msg.id
                  << std::dec << std::endl;
      }
      tx_queue.pop();
    } while (!tx_queue.empty());
  }
  std::cout << "[SEND] Exiting Send Loop" << std::endl;
}

void recvLoop(socketcan::CANSocket *socket) {
  uint32_t id;
  std::vector<uint8_t> data;
  uint64_t timestamp;
  socketcan::CanMessage msg;

  while (running) {
    if (socket->receiveMessage(id, data, timestamp)) {
      msg = socketcan::CanMessage(id, data, timestamp);
      if (std::find(RX_IDS.begin(), RX_IDS.end(), msg.id) != RX_IDS.end()) {
        msg.print();
      }
    } else if (errno == ENOBUFS || errno == ENETDOWN) {
      std::cerr << "[RECV] Socket error, restarting...\n";
      socket->close();
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      socket->initialize();
    }
  }
}

void socketcan_tranceiver() {
  while (running) {
    for (auto &msg : {
             socketcan::CanMessage(0x100, {0, 1, 2, 3, 4, 5, 6, 7}),
             socketcan::CanMessage(0x101, {1, 2, 3, 4, 5, 6, 7, 8}),
             socketcan::CanMessage(0x102, {2, 3, 4, 5, 6, 7, 8, 9}),
         }) {
      tx_queue_message_push(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  std::cout << "Run Goodbye" << std::endl;
}

void signalHandler(int) {
  running = false;
  std::cout << "Stopping...\n";
}

int main() {
  std::signal(SIGINT, signalHandler);

  socketcan::CANSocket socket("vcan0");
  if (!socket.initialize()) {
    std::cerr << "Failed to initialize CAN interface\n";
    return 1;
  }

  // std::thread rx(recvLoop, &socket);
  std::thread tx(sendLoop, &socket);
  std::thread runloop(socketcan_tranceiver);

  std::cout << "Running... Press Enter to exit.\n";
  std::cin.get();
  running = false;

  tx.join();
  runloop.join();

  // rx.join();
  return 0;
}
