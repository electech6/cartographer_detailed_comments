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

#ifndef CARTOGRAPHER_COMMON_RATE_TIMER_H_
#define CARTOGRAPHER_COMMON_RATE_TIMER_H_

#include <chrono>
#include <deque>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"

namespace cartographer {
namespace common {

// Computes the rate at which pulses come in.
template <typename ClockType = std::chrono::steady_clock>
/**
 * @brief 该类是脉冲频率计数类，计算在一段时间内的脉冲率
 */
class RateTimer {
 public:
  // Computes the rate at which pulses come in over 'window_duration' in wall
  // time.
  /**
   * @brief Construct a new Rate Timer object 
   * 并初始化Duration对象
   * @param[in] window_duration
   */
  explicit RateTimer(const common::Duration window_duration)
      : window_duration_(window_duration) {}
  ~RateTimer() {}
  //delete关键字，表示删除该构造函数，防止其他子类调用
  RateTimer(const RateTimer&) = delete;
  RateTimer& operator=(const RateTimer&) = delete;

  // Returns the pulse rate in Hz.
  /**
   * @brief 计算频率
   * @return double 
   */
  double ComputeRate() const {
    if (events_.empty()) {
      return 0.;
    }
    //队列大小除以队列时间=频率
    // ? 为什么size()-1
    return static_cast<double>(events_.size() - 1) /
           common::ToSeconds((events_.back().time - events_.front().time));
  }

  // Returns the ratio of the pulse rate (with supplied times) to the wall time
  // rate. For example, if a sensor produces pulses at 10 Hz, but we call Pulse
  // at 20 Hz wall time, this will return 2.
  /**
   * @brief 计算系统时间频率比率
   * @return double 
   */
  double ComputeWallTimeRateRatio() const {
    if (events_.empty()) {
      return 0.;
    }
    return common::ToSeconds((events_.back().time - events_.front().time)) /
           common::ToSeconds(events_.back().wall_time -
                             events_.front().wall_time);
  }

  // Records an event that will contribute to the computed rate.
  /**
   * @brief 产生脉冲
   * @param[in] time 
   */
  void Pulse(common::Time time) {
    //将一个event结构体压入队列
    events_.push_back(Event{time, ClockType::now()});
    while (events_.size() > 2 &&
           (events_.back().wall_time - events_.front().wall_time) >
               window_duration_) {
      events_.pop_front();
    }
  }

  // Returns a debug string representation.
  /**
   * @brief 
   * @return std::string 返回调试字符串表示形式。
   */
  std::string DebugString() const {
    if (events_.size() < 2) {
      return "unknown";
    }
    std::ostringstream out;
    //std::fixed 与std::setprecision合作保留后面几位小数
    //std::setprecision(2)  控制输出流显示浮点数的数字个数2
    out << std::fixed << std::setprecision(2) << ComputeRate() << " Hz "
        << DeltasDebugString() << " (pulsed at "
        << ComputeWallTimeRateRatio() * 100. << "% real time)";
    return out.str();
  }

 private:
 /**
  * @brief 时间结构体
  */
  struct Event {
    common::Time time;
    typename ClockType::time_point wall_time;
  };

  // Computes all differences in seconds between consecutive pulses.
  /**
   * @brief 计算连续脉冲之间以秒为单位的所有差异。
   * @return std::vector<double> 
   */
  std::vector<double> ComputeDeltasInSeconds() const {
    CHECK_GT(events_.size(), 1);
    const size_t count = events_.size() - 1;
    std::vector<double> result;
    //分配容器空间
    result.reserve(count);
    for (size_t i = 0; i != count; ++i) {
      result.push_back(
          common::ToSeconds(events_[i + 1].time - events_[i].time));
    }
    return result;
  }

  // Returns the average and standard deviation of the deltas.
  /**
   * @brief 返回增量的平均值和标准偏差。
   * @return std::string 
   */
  std::string DeltasDebugString() const {
    const auto deltas = ComputeDeltasInSeconds();
    // 计算容器deltas里面元素之和
    const double sum = std::accumulate(deltas.begin(), deltas.end(), 0.);
    // 增量的平均值
    const double mean = sum / deltas.size();

    double squared_sum = 0.;
    /**
     * @brief 计算方差
     */
    for (const double x : deltas) {
      squared_sum += common::Pow2(x - mean);
    }
    // 计算标准差
    const double sigma = std::sqrt(squared_sum / (deltas.size() - 1));

    std::ostringstream out;
    out << std::scientific << std::setprecision(2) << mean << " s +/- " << sigma
        << " s";
    return out.str();
  }
  //创建一个队列events_
  std::deque<Event> events_;
  //创建只可读Duation对象window_duration_
  const common::Duration window_duration_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_RATE_TIMER_H_
