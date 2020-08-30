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

#ifndef CARTOGRAPHER_SENSOR_POINT_CLOUD_H_
#define CARTOGRAPHER_SENSOR_POINT_CLOUD_H_

#include <vector>

#include "Eigen/Core"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/sensor/rangefinder_point.h"
#include "cartographer/transform/rigid_transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {

// Stores 3D positions of points.
// For 2D points, the third entry is 0.f.
/**
 * @brief 保存3D点云的位姿，如果是2d，则第三个元素为0
 */
using PointCloud = std::vector<RangefinderPoint>;

// Stores 3D positions of points with their relative measurement time in the
// fourth entry. Time is in seconds, increasing and relative to the moment when
// the last point was acquired. So, the fourth entry for the last point is 0.f.
// If timing is not available, all fourth entries are 0.f. For 2D points, the
// third entry is 0.f (and the fourth entry is time).
/**
 * @brief 加入时间这个变量，放在第4个元素上
 */
using TimedPointCloud = std::vector<TimedRangefinderPoint>;
/**
 * @brief 点云+时间+光强度的结构体，里面有两个容器，一个点云+时间，一个光强度
 */
struct PointCloudWithIntensities {
  // 包含时间的点云容器
  TimedPointCloud points;
  std::vector<float> intensities;
};

// Transforms 'point_cloud' according to 'transform'.
/**
 * @brief 根据转换矩阵来转换点云
 * @param[in] point_cloud 
 * @param[in] transform 
 * @return PointCloud 
 */
PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform);

// Transforms 'point_cloud' according to 'transform'.
/**
 * @brief 根据转换矩阵来转换点云+时间
 * @param[in] point_cloud 
 * @param[in] transform 
 * @return TimedPointCloud 
 */
TimedPointCloud TransformTimedPointCloud(const TimedPointCloud& point_cloud,
                                         const transform::Rigid3f& transform);

// Returns a new point cloud without points that fall outside the region defined
// by 'min_z' and 'max_z'.
/**
 * @brief 去除Z轴范围之外的点云变成新的一个点云
 * @param[in] point_cloud 
 * @param[in] min_z 
 * @param[in] max_z 
 * @return PointCloud 
 */
PointCloud CropPointCloud(const PointCloud& point_cloud, float min_z,
                          float max_z);

// Returns a new point cloud without points that fall outside the region defined
// by 'min_z' and 'max_z'.
/**
 * @brief 去除Z轴范围之外的点云变成新的一个点云+时间
 * @param[in] point_cloud 
 * @param[in] min_z 
 * @param[in] max_z 
 * @return TimedPointCloud 
 */
TimedPointCloud CropTimedPointCloud(const TimedPointCloud& point_cloud,
                                    float min_z, float max_z);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_POINT_CLOUD_H_
