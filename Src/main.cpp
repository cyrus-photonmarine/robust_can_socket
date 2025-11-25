#include "CANSocket.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <iostream>
#include <mutex>
#include <optional>
#include <queue>
#include <thread>
#include <vector>

std::array<uint32_t, 8> RX_IDS = {0x101, 0x102, 0x103, 0x201,
                                  0x202, 0x203, 0x301, 0x302};

std::atomic<bool> running{false};
socketcan::Transmitter can_transmitter("vcan0");
void socketcan_tranceiver(const std::vector<socketcan::CanMessage> &msgs) {
  while (running) {
    for (const auto &msg : msgs) {
      can_transmitter.send(std::move(msg));
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

  const std::vector<socketcan::CanMessage> msgs = {
      socketcan::CanMessage(0x100, {0, 1, 2, 3, 4, 5, 6, 7}),
      socketcan::CanMessage(0x101, {1, 2, 3, 4, 5, 6, 7, 8}),
      socketcan::CanMessage(0x102, {2, 3, 4, 5, 6, 7, 8, 9}),
  };

  can_transmitter.start();
  running = true;
  std::thread send_loop(socketcan_tranceiver, msgs);
  std::cout << "Running... Press Enter to exit.\n";
  std::cin.get();
  running = false;
  can_transmitter.stop();
  send_loop.join();

  return 0;
}
