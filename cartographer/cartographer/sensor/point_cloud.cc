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

#include "cartographer/sensor/point_cloud.h"

#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace sensor {
/**
 * @brief 根据3D转换，变成新的点云
 * @param[in] point_cloud 
 * @param[in] transform 
 * @return PointCloud 
 */
PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform) {
  PointCloud result;
  //预留点云大小的空间
  result.reserve(point_cloud.size());
  for (const RangefinderPoint& point : point_cloud) {
    //将转换后的点云压入容器
    result.emplace_back(transform * point);
  }
  //返回新的点云数据
  return result;
}
/**
 * @brief 和上面的TransformPointCloud类似，只是容器加了一个时间的变量
 * @param[in] point_cloud 
 * @param[in] transform 
 * @return TimedPointCloud 
 */
TimedPointCloud TransformTimedPointCloud(const TimedPointCloud& point_cloud,
                                         const transform::Rigid3f& transform) {
  TimedPointCloud result;
  result.reserve(point_cloud.size());
  for (const TimedRangefinderPoint& point : point_cloud) {
    result.push_back(transform * point);
  }
  return result;
}
/**
 * @brief 裁剪Z轴范围的点云
 * @param[in] point_cloud 
 * @param[in] min_z 
 * @param[in] max_z 
 * @return PointCloud 
 */
PointCloud CropPointCloud(const PointCloud& point_cloud, const float min_z,
                          const float max_z) {
  PointCloud cropped_point_cloud;
  for (const RangefinderPoint& point : point_cloud) {
    if (min_z <= point.position.z() && point.position.z() <= max_z) {
      cropped_point_cloud.push_back(point);
    }
  }
  return cropped_point_cloud;
}
/**
 * @brief 和上面的CropPointCloud差不多，只是容器加了一个时间的变量
 * @param[in] point_cloud 
 * @param[in] min_z 
 * @param[in] max_z 
 * @return TimedPointCloud 
 */
TimedPointCloud CropTimedPointCloud(const TimedPointCloud& point_cloud,
                                    const float min_z, const float max_z) {
  TimedPointCloud cropped_point_cloud;
  for (const TimedRangefinderPoint& point : point_cloud) {
    if (min_z <= point.position.z() && point.position.z() <= max_z) {
      cropped_point_cloud.push_back(point);
    }
  }
  return cropped_point_cloud;
}

}  // namespace sensor
}  // namespace cartographer
