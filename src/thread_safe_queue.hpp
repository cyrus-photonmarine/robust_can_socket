#ifndef THREAD_SAFE_QUEUE_H_
#define THREAD_SAFE_QUEUE_H_

#include <mutex>
#include <queue>
#include <optional>
#include <chrono>
#include <condition_variable>

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


#endif // THREAD_SAFE_QUEUE_H_
