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

#ifndef CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_

#include <functional>
#include <memory>
#include <string>

#include "absl/memory/memory.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/sensor/fixed_frame_pose_data.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

namespace cartographer {
namespace mapping {

proto::TrajectoryBuilderOptions CreateTrajectoryBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

class LocalSlamResultData;

// This interface is used for both 2D and 3D SLAM. Implementations wire up a
// global SLAM stack, i.e. local SLAM for initial pose estimates, scan matching
// to detect loop closure, and a sparse pose graph optimization to compute
// optimized pose estimates.
class TrajectoryBuilderInterface {
 public:
 /**
  * @brief 就是用来保存插入 Local Slam 的一个节点的数据结构
  */
  struct InsertionResult {
    NodeId node_id;
    std::shared_ptr<const TrajectoryNode::Data> constant_data; //TrajectoryNode::Data 包含了经过处理的一帧传感器数据
    std::vector<std::shared_ptr<const Submap>> insertion_submaps;//已经建立起来的子图列表。
  };

  // A callback which is called after local SLAM processes an accumulated
  // 'sensor::RangeData'. If the data was inserted into a submap, reports the
  // assigned 'NodeId', otherwise 'nullptr' if the data was filtered out.
  /**
   * @brief 在本地SLAM处理累积的'sensor :: RangeData'之后调用的回调。 
   * 如果将数据插入到子图中，则报告分配的“ NodeId”，否则报告“ nullptr”（如果已滤除数据）。
   */
  using LocalSlamResultCallback =
      std::function<void(int /* trajectory ID */, common::Time,//时间
                         transform::Rigid3d /* local pose estimate */,
                         sensor::RangeData /* in local frame */,
                         std::unique_ptr<const InsertionResult>)>;
/**
 * @brief 定义传感器ID
 */
  struct SensorId {
    enum class SensorType {
      RANGE = 0,
      IMU,
      ODOMETRY,
      FIXED_FRAME_POSE,
      LANDMARK,
      LOCAL_SLAM_RESULT
    };
    //传感器类型
    SensorType type;
    //传感器ID
    std::string id;

    bool operator==(const SensorId& other) const {
      return std::forward_as_tuple(type, id) ==
             std::forward_as_tuple(other.type, other.id);
    }

    bool operator<(const SensorId& other) const {
      return std::forward_as_tuple(type, id) <
             std::forward_as_tuple(other.type, other.id);
    }
  };

  TrajectoryBuilderInterface() {}
  virtual ~TrajectoryBuilderInterface() {}

  TrajectoryBuilderInterface(const TrajectoryBuilderInterface&) = delete;
  TrajectoryBuilderInterface& operator=(const TrajectoryBuilderInterface&) =
      delete;
  // 处理传感器数据的 5 个纯虚函数
  virtual void AddSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data) = 0;
  virtual void AddSensorData(const std::string& sensor_id,
                             const sensor::ImuData& imu_data) = 0;
  virtual void AddSensorData(const std::string& sensor_id,
                             const sensor::OdometryData& odometry_data) = 0;
  virtual void AddSensorData(
      const std::string& sensor_id,
      const sensor::FixedFramePoseData& fixed_frame_pose) = 0;
  virtual void AddSensorData(const std::string& sensor_id,
                             const sensor::LandmarkData& landmark_data) = 0;
  // Allows to directly add local SLAM results to the 'PoseGraph'. Note that it
  // is invalid to add local SLAM results for a trajectory that has a
  // 'LocalTrajectoryBuilder2D/3D'.
  //直接将 Local Slam 的结果添加到 PoseGraph 的函数
  virtual void AddLocalSlamResultData(
      std::unique_ptr<mapping::LocalSlamResultData> local_slam_result_data) = 0;
};

proto::SensorId ToProto(const TrajectoryBuilderInterface::SensorId& sensor_id);
TrajectoryBuilderInterface::SensorId FromProto(
    const proto::SensorId& sensor_id_proto);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_BUILDER_INTERFACE_H_
