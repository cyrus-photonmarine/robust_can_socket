#include "CANSocket.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <queue>
#include <thread>
#include <vector>

// Loop through a given vector of CAN messages in thread.
std::atomic<bool> running{false};
socketcan::Transmitter can_transmitter("vcan0");
void socketcan_tranceiver(const std::vector<socketcan::CanMessage> &msgs) {
  while (running) {
    for (const auto &msg : msgs) {
      can_transmitter.send(std::move(msg));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
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

  /* This list of CAN messages will be sent repeatedly */
  const std::vector<socketcan::CanMessage> msgs = {
      socketcan::CanMessage(0x100, {0, 1, 2, 3, 4, 5, 6, 7}),
      socketcan::CanMessage(0x101, {1, 2, 3, 4, 5, 6, 7, 8}),
      socketcan::CanMessage(0x102, {2, 3, 4, 5, 6, 7, 8, 9}),
  };

  /* Fire up the transmit queue service */
  can_transmitter.start();
  running = true;

  /* Fire up the message-producer service */
  std::thread send_loop(socketcan_tranceiver, msgs);

  /* Block until exit requested by user. */
  std::cout << "Running... Press Enter to exit.\n";
  std::cin.get();

  /* Break running loops and exit all threads. */
  running = false;
  can_transmitter.stop();
  send_loop.join();

  return 0;
}
