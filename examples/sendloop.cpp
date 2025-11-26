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

// Set up a test loop in hardware - CAN1 receives from CAN0.
socketcan::Transmitter can_transmitter("can0"), can_listener("can1");

// CAN0 sends messages in a loop.
void socketcan_tranceiver(const std::vector<socketcan::CanMessage> &msgs) {
  while (running) {
    for (const auto &msg : msgs) {
      can_transmitter.send(std::move(msg));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  std::cout << "Exiting Send Thread." << std::endl;
}

// Listen for messages received on CAN1.  Should receive what was sent on CAN0.
void socketcan_listener() {
  socketcan::CanMessage msg;
  while (running) {
    if (can_listener.receive(msg)) {
      std::cout << "Received: " << std::endl;
      msg.print();
    }
  }
  std::cout << "Exiting Listener Thread." << std::endl;
}

// Nuke it from high orbit. Only way to be sure.
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

  /* Fire up the transmit queue service on CAN0 */
  can_transmitter.start();

  /* Set up a listener service on CAN1 */
  can_listener.start();

  running = true;

  /* Fire up the message-producer service CAN0 */
  std::thread send_loop(socketcan_tranceiver, msgs);

  /* Monitor receive queue on CAN1 */
  std::thread recv_loop(socketcan_listener);

  /* Block until exit requested by user. */
  std::cout << "Running... Press Enter to exit.\n";
  std::cin.get();

  /* Break running loops and exit all threads. */
  running = false;
  can_transmitter.stop();
  can_listener.stop();
  send_loop.join();
  recv_loop.join();

  return 0;
}
