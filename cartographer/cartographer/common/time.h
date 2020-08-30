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

#ifndef CARTOGRAPHER_COMMON_TIME_H_
#define CARTOGRAPHER_COMMON_TIME_H_

#include <chrono>
#include <ostream>
#include <ratio>

#include "cartographer/common/port.h"
/**
 * @brief 主要功能是提供时间转换函数
 */
namespace cartographer {
namespace common {
//719162 是0001年1月1日到1970年1月1日所经历的天数
constexpr int64 kUtsEpochOffsetFromUnixEpochInSeconds =
    (719162ll * 24ll * 60ll * 60ll);
/**
 * @brief 对时间概念的一些别称
 */
struct UniversalTimeScaleClock {
  using rep = int64;
  //代表0.1us
  using period = std::ratio<1, 10000000>;
  //表示时间间隔
  using duration = std::chrono::duration<rep, period>;
  //使用别称
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  static constexpr bool is_steady = true;
};

// Represents Universal Time Scale durations and timestamps which are 64-bit
// integers representing the 100 nanosecond ticks since the Epoch which is
// January 1, 1 at the start of day in UTC.
// 微秒,us
using Duration = UniversalTimeScaleClock::duration;
// 时间点
using Time = UniversalTimeScaleClock::time_point;

// Convenience functions to create common::Durations.
//将秒数seconds转为c++的duration实例对象
Duration FromSeconds(double seconds);
//将毫秒milliseconds转为c++的duration实例对象
Duration FromMilliseconds(int64 milliseconds);

// Returns the given duration in seconds.
// ? 这两者的区别
// ? 用std::chrono::duration表示一段时间
// ? steady_clock 是稳定的时钟，用来计算时间间隔的。
//将的duration实例对象转为 秒数 
double ToSeconds(Duration duration);
// ! 旧版本没有
// steady_clock：高精度时间，用在需要得到时间间隔，并且这个时间间隔不会因为修改系统时间而受影响的场景
double ToSeconds(std::chrono::steady_clock::duration duration);

// Creates a time from a Universal Time Scale.
//将TUC时间(微秒)转化为c++的time_point对象
Time FromUniversal(int64 ticks);

// Outputs the Universal Time Scale timestamp for a given Time.
//将c++的time_point对象转为TUC时间,单位是us
int64 ToUniversal(Time time);

// For logging and unit tests, outputs the timestamp integer.
// 重载<<操作符,将time_point以string输出
std::ostream& operator<<(std::ostream& os, Time time);

// CPU time consumed by the thread so far, in seconds.
//计算进程中的CPU时间
// ! 旧版本没有
double GetThreadCpuTimeSeconds();

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_TIME_H_
