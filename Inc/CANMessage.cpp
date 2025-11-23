#include "CANMessage.h"
#include <cstdint>
#include <vector>

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
