/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_SENSOR_RANGEFINDER_POINT_H_
#define CARTOGRAPHER_SENSOR_RANGEFINDER_POINT_H_

#include <vector>

#include "Eigen/Core"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {

// Stores 3D position of a point observed by a rangefinder sensor.
//  3D(x,y,z)点云结构体
struct RangefinderPoint {
  Eigen::Vector3f position;
};

// Stores 3D position of a point with its relative measurement time.
// See point_cloud.h for more details.
//点云+时间(浮点型)
struct TimedRangefinderPoint {
  Eigen::Vector3f position;
  float time;
};
/**
 * @brief 重写*运算，返回点云的结构体。内联函数，加快运算
 * @param[in] lhs 
 * @param[in] rhs 
 * @return RangefinderPoint 
 */
template <class T>
inline RangefinderPoint operator*(const transform::Rigid3<T>& lhs,
                                  const RangefinderPoint& rhs) {
  RangefinderPoint result = rhs;
  result.position = lhs * rhs.position;
  return result;
}
/**
 * @brief 重写*运算，返回点云+时间的结构体，内联函数，加快运算
 * @param[in] lhs 
 * @param[in] rhs 
 * @return TimedRangefinderPoint 
 */
template <class T>
inline TimedRangefinderPoint operator*(const transform::Rigid3<T>& lhs,
                                       const TimedRangefinderPoint& rhs) {
  TimedRangefinderPoint result = rhs;
  result.position = lhs * rhs.position;
  return result;
}
/**
 * @brief 重写==符号
 * @param[in] lhs RangefinderPoint类型
 * @param[in] rhs RangefinderPoint类型
 * @return true 
 * @return false 
 */
inline bool operator==(const RangefinderPoint& lhs,
                       const RangefinderPoint& rhs) {
  return lhs.position == rhs.position;
}
/**
 * @brief 重写==符号
 * @param[in] lhs 
 * @param[in] rhs 
 * @return true 
 * @return false 
 */
inline bool operator==(const TimedRangefinderPoint& lhs,
                       const TimedRangefinderPoint& rhs) {
  return lhs.position == rhs.position && lhs.time == rhs.time;
}
/**
 * @brief 3D反序列化
 * @param[in] rangefinder_point_proto 
 * @return RangefinderPoint 
 */
inline RangefinderPoint FromProto(
    const proto::RangefinderPoint& rangefinder_point_proto) {
  return {transform::ToEigen(rangefinder_point_proto.position())};
}
/**
 * @brief 3D点云序列化
 * @param[in] rangefinder_point 
 * @return proto::RangefinderPoint 
 */
inline proto::RangefinderPoint ToProto(
    const RangefinderPoint& rangefinder_point) {
  proto::RangefinderPoint proto;
  *proto.mutable_position() = transform::ToProto(rangefinder_point.position);
  return proto;
}
/**
 * @brief 3D点云+时间反序列化
 * @param[in] timed_rangefinder_point_proto 
 * @return TimedRangefinderPoint 
 */
inline TimedRangefinderPoint FromProto(
    const proto::TimedRangefinderPoint& timed_rangefinder_point_proto) {
  return {transform::ToEigen(timed_rangefinder_point_proto.position()),
          timed_rangefinder_point_proto.time()};
}
/**
 * @brief 3D点云+时间序列化
 * @param[in] timed_rangefinder_point 
 * @return proto::TimedRangefinderPoint 
 */
inline proto::TimedRangefinderPoint ToProto(
    const TimedRangefinderPoint& timed_rangefinder_point) {
  proto::TimedRangefinderPoint proto;
  *proto.mutable_position() =
      transform::ToProto(timed_rangefinder_point.position);
  proto.set_time(timed_rangefinder_point.time);
  return proto;
}
/**
 * @brief TimedRangefinderPoint转成RangefinderPoint
 * @param[in] timed_rangefinder_point 
 * @return RangefinderPoint 
 */
inline RangefinderPoint ToRangefinderPoint(
    const TimedRangefinderPoint& timed_rangefinder_point) {
  return {timed_rangefinder_point.position};
}
/**
 * @brief RangefinderPoint+time转成TimedRangefinderPoint
 * @param[in] rangefinder_point 
 * @param[in] time 
 * @return TimedRangefinderPoint 
 */
inline TimedRangefinderPoint ToTimedRangefinderPoint(
    const RangefinderPoint& rangefinder_point, const float time) {
  return {rangefinder_point.position, time};
}

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_RANGEFINDER_POINT_H_
