/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_COMMON_BLOCKING_QUEUE_H_
#define CARTOGRAPHER_COMMON_BLOCKING_QUEUE_H_

#include <cstddef>
#include <deque>
#include <memory>

#include "absl/synchronization/mutex.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "glog/logging.h"

namespace cartographer {
namespace common {

// A thread-safe blocking queue that is useful for producer/consumer patterns.
// 'T' must be movable.
template <typename T>
/**
 * @brief 
 * 构造函数  BlockingQueue(kInfiniteQueueSize)  BlockingQueue(const size_t queue_size) 
 */
class BlockingQueue {
 public:
  static constexpr size_t kInfiniteQueueSize = 0;

  // Constructs a blocking queue with infinite queue size.
  /**
   * @brief Construct a new Blocking Queue object
   */
  BlockingQueue() : BlockingQueue(kInfiniteQueueSize) {}

  BlockingQueue(const BlockingQueue&) = delete;
  BlockingQueue& operator=(const BlockingQueue&) = delete;

  // Constructs a blocking queue with a size of 'queue_size'.
  /**
   * @brief Construct a new Blocking Queue object 初始化queue
   * @param[in] queue_size 
   */
  explicit BlockingQueue(const size_t queue_size) : queue_size_(queue_size) {}

  // Pushes a value onto the queue. Blocks if the queue is full.
  void Push(T t) {
    // 调用predicate必须加锁mutex_
    const auto predicate = [this]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
      return QueueNotFullCondition();
    };
    //加锁
    absl::MutexLock lock(&mutex_);
    mutex_.Await(absl::Condition(&predicate));
    //右值压入队列
    deque_.push_back(std::move(t));
  }

  // Like push, but returns false if 'timeout' is reached.
  /**
   * @brief 最多等待timeout时间
   * @param[in] t 
   * @param[in] timeout 
   * @return true 
   * @return false 
   */
  bool PushWithTimeout(T t, const common::Duration timeout) {
    const auto predicate = [this]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
      return QueueNotFullCondition();
    };
    absl::MutexLock lock(&mutex_);
    if (!mutex_.AwaitWithTimeout(absl::Condition(&predicate),
                                 absl::FromChrono(timeout))) {
      return false;
    }
    deque_.push_back(std::move(t));
    return true;
  }

  // Pops the next value from the queue. Blocks until a value is available.
  /**
   * @brief 从队列中弹出下一个值。 阻塞直到值可用。
   * @return T 
   */
  T Pop() {
    const auto predicate = [this]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
      return !QueueEmptyCondition();
    };
    absl::MutexLock lock(&mutex_);
    mutex_.Await(absl::Condition(&predicate));

    T t = std::move(deque_.front());
    deque_.pop_front();
    return t;
  }

  // Like Pop, but can timeout. Returns nullptr in this case.
  /**
   * @brief 超时则返回空指针
   * @param[in] timeout 
   * @return T 
   */
  T PopWithTimeout(const common::Duration timeout) {
    const auto predicate = [this]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
      return !QueueEmptyCondition();
    };
    absl::MutexLock lock(&mutex_);
    if (!mutex_.AwaitWithTimeout(absl::Condition(&predicate),
                                 absl::FromChrono(timeout))) {
      return nullptr;
    }
    T t = std::move(deque_.front());
    deque_.pop_front();
    return t;
  }

  // Like Peek, but can timeout. Returns nullptr in this case.
  /**
   * @brief 
   * @param[in] timeout 
   * @return R* 
   */
  template <typename R>
  R* PeekWithTimeout(const common::Duration timeout) {
    const auto predicate = [this]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
      return !QueueEmptyCondition();
    };
    absl::MutexLock lock(&mutex_);
    if (!mutex_.AwaitWithTimeout(absl::Condition(&predicate),
                                 absl::FromChrono(timeout))) {
      return nullptr;
    }
    return deque_.front().get();
  }

  // Returns the next value in the queue or nullptr if the queue is empty.
  // Maintains ownership. This assumes a member function get() that returns
  // a pointer to the given type R.
  template <typename R>
  /**
   * @brief 
   * @return const R* 
   */
  const R* Peek() {
    absl::MutexLock lock(&mutex_);
    if (deque_.empty()) {
      return nullptr;
    }
    // ? .get()的用法
    return deque_.front().get();
  }

  // Returns the number of items currently in the queue.
  /**
   * @brief 封装队列大小，然后加锁
   * @return size_t 
   */
  size_t Size() {
    absl::MutexLock lock(&mutex_);
    return deque_.size();
  }

  // Blocks until the queue is empty.
  /**
   * @brief 等待直到队列为空
   */
  void WaitUntilEmpty() {
    const auto predicate = [this]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
      return QueueEmptyCondition();
    };
    absl::MutexLock lock(&mutex_);
    mutex_.Await(absl::Condition(&predicate));
  }

 private:
  // Returns true iff the queue is empty.
  /**
   * @brief 返回ture，如何队列为空
   */
  bool QueueEmptyCondition() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
    return deque_.empty();
  }

  // Returns true iff the queue is not full.
  /**
   * @brief 判断队列是否为满状态
   */
  bool QueueNotFullCondition() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
    return queue_size_ == kInfiniteQueueSize || deque_.size() < queue_size_;
  }

  absl::Mutex mutex_;
  //队列大小
  const size_t queue_size_ GUARDED_BY(mutex_);
  //创建队列
  std::deque<T> deque_ GUARDED_BY(mutex_);
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_BLOCKING_QUEUE_H_
