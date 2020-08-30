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

#include "cartographer/common/time.h"

#include <time.h>

#include <cerrno>
#include <cstring>
#include <string>

#include "glog/logging.h"

namespace cartographer {
namespace common {
/**
 * @brief 将秒数seconds转为c++的duration实例对象
 * @param  seconds
 * @return Duration 
 */
Duration FromSeconds(const double seconds) {
  // duration_cast是c++ 11的时间显式转换函数.
  return std::chrono::duration_cast<Duration>(
    //将double类型的秒数转化为duration对象
      std::chrono::duration<double>(seconds));
}
/**
 * @brief 将的duration实例对象转为 秒数
 * @param  duration
 * @return double 
 */
double ToSeconds(const Duration duration) {
  //反转化,count()返回时钟周期数,ticks
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration)
      .count();
}
/**
 * @brief 将高精度duration实例对象转为 秒数
 * @param  duration
 * @return double 
 */
double ToSeconds(const std::chrono::steady_clock::duration duration) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration)
      .count();
}
/**
 * @brief //将TUC时间(微秒)转化为c++的time_point对象
 * @param  ticks
 * @return Time 
 */
//先构造一个临时duration对象,再将其转化为time_point对象
//Duration(ticks)调用的是UniversalTimeScaleClock的构造函数  
Time FromUniversal(const int64 ticks) { return Time(Duration(ticks)); }
/**
 * @brief 将c++的time_point对象转为TUC时间,单位是us
 * @param  time
 * @return int64 
 */
//count()返回time_point自epoch以来的时钟周期数
int64 ToUniversal(const Time time) { return time.time_since_epoch().count(); }
/**
 * @brief 重载<<操作符,将time_point以string输出
 * @param  os
 * @param  time
 * @return std::ostream& 
 */
//先将Time转化为 int64 , 再转为字符串形式
std::ostream& operator<<(std::ostream& os, const Time time) {
  os << std::to_string(ToUniversal(time));
  return os;
}
/**
 * @brief 将毫秒milliseconds转为c++的duration实例对象
 * @param  milliseconds
 * @return common::Duration 
 */
common::Duration FromMilliseconds(const int64 milliseconds) {
  return std::chrono::duration_cast<Duration>(
      std::chrono::milliseconds(milliseconds));
}
/**
 * @brief Get the Thread Cpu Time Seconds object
 * @return double 
 */
double GetThreadCpuTimeSeconds() {
#ifndef WIN32
  struct timespec thread_cpu_time;
  CHECK(clock_gettime(CLOCK_THREAD_CPUTIME_ID, &thread_cpu_time) == 0)
      << std::strerror(errno);
  return thread_cpu_time.tv_sec + 1e-9 * thread_cpu_time.tv_nsec;
#else
  return 0.;
#endif
}

}  // namespace common
}  // namespace cartographer
