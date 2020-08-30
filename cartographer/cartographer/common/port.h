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

#ifndef CARTOGRAPHER_COMMON_PORT_H_
#define CARTOGRAPHER_COMMON_PORT_H_

#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/gzip.hpp>  //包含多种解压与压缩算法
#include <boost/iostreams/filtering_stream.hpp>  //配合filter实现流过滤
#include <cinttypes>
#include <cmath>
#include <string>
/**
 * @brief  common-port.h文件主要实现2大功能：
         1，使用std::lround对浮点数进行四舍五入取整运算
         2，利用boost的iostreams/filter/gzip对字符串压缩与解压缩
 */
namespace cartographer {

using int8 = int8_t;
using int16 = int16_t;
using int32 = int32_t;
using int64 = int64_t;
using uint8 = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;

namespace common {
/**
 * @brief
 * @param  x   浮点型常数
 * @return int 调用std::lround
 */
inline int RoundToInt(const float x) { return std::lround(x); }
/**
 * @brief
 * @param  x    double型常数
 * @return int  调用std::lround
 */
inline int RoundToInt(const double x) { return std::lround(x); }
/**
 * @brief
 * @param  x    浮点型常数
 * @return int64 调用std::lround
 */
inline int64 RoundToInt64(const float x) { return std::lround(x); }
/**
 * @brief
 * @param  x    double型常数
 * @return int64    调用std::lround
 */

inline int64 RoundToInt64(const double x) { return std::lround(x); }
/**
 * @brief   对字符串进行压缩
 * @param  uncompressed    未压缩的字符串
 * @param  compressed       完成压缩的字符串
 */
inline void FastGzipString(const std::string& uncompressed,
                           std::string* compressed) {
  //创建过滤流
  boost::iostreams::filtering_ostream out;
  //使用快速压缩算法
  out.push(
      boost::iostreams::gzip_compressor(boost::iostreams::zlib::best_speed));
  //对compressed使用后插迭代器
  out.push(boost::iostreams::back_inserter(*compressed));
  //压缩 char *,插入compressed
  boost::iostreams::write(out,
                          reinterpret_cast<const char*>(uncompressed.data()),
                          uncompressed.size());
}
/**
 * @brief   利用gzip_decompressor解压缩string
 * @param  compressed   待解压的字符串
 * @param  decompressed 完成解压的字符串
 */
inline void FastGunzipString(const std::string& compressed,
                             std::string* decompressed) {
  boost::iostreams::filtering_ostream out;
  out.push(boost::iostreams::gzip_decompressor());
  out.push(boost::iostreams::back_inserter(*decompressed));
  boost::iostreams::write(out, reinterpret_cast<const char*>(compressed.data()),
                          compressed.size());
}

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_PORT_H_
