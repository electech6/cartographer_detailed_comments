/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_
#define CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_

#include "Eigen/Core"
#include "cartographer/common/time.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace sensor {
/**
 * @brief 点云数据时间和系统时间放在一起的结构体
 */
struct TimedPointCloudData {
  common::Time time;
  Eigen::Vector3f origin;
  TimedPointCloud ranges;
};
/**
 * @brief 和上面结构体相比，多了初始的序号，之间的关系，相当于集合和个体
 */
struct TimedPointCloudOriginData {
  //点云+时间+原始序号
  struct RangeMeasurement {
    TimedRangefinderPoint point_time;
    size_t origin_index;
  };
  //系统时间
  common::Time time;
  std::vector<Eigen::Vector3f> origins;
  std::vector<RangeMeasurement> ranges;
};

// Converts 'timed_point_cloud_data' to a proto::TimedPointCloudData.
/**
 * @brief 序列化
 * @param[in] timed_point_cloud_data 
 * @return proto::TimedPointCloudData 
 */
proto::TimedPointCloudData ToProto(
    const TimedPointCloudData& timed_point_cloud_data);

// Converts 'proto' to TimedPointCloudData.
/**
 * @brief 反序列化
 * @param[in] proto 
 * @return TimedPointCloudData 
 */
TimedPointCloudData FromProto(const proto::TimedPointCloudData& proto);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_
