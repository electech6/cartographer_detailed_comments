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

#ifndef CARTOGRAPHER_MAPPING_POSE_GRAPH_INTERFACE_H_
#define CARTOGRAPHER_MAPPING_POSE_GRAPH_INTERFACE_H_

#include <chrono>
#include <vector>

#include "absl/types/optional.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

class PoseGraphInterface {
 public:
  // A "constraint" as in the paper by Konolige, Kurt, et al. "Efficient sparse
  // pose adjustment for 2d mapping." Intelligent Robots and Systems (IROS),
  // 2010 IEEE/RSJ International Conference on (pp. 22--29). IEEE, 2010.
  /**
   * @brief 约束结构
   */
  struct Constraint {
    struct Pose {
      //相对位姿
      transform::Rigid3d zbar_ij;
      //translation 的权重
      double translation_weight;
      //rotation 的权重
      double rotation_weight;
    };

    SubmapId submap_id;  // 'i' in the paper.
    NodeId node_id;      // 'j' in the paper.

    // Pose of the node 'j' relative to submap 'i'.
    Pose pose;

    // Differentiates between intra-submap (where node 'j' was inserted into
    // submap 'i') and inter-submap constraints (where node 'j' was not inserted
    // into submap 'i').
    enum Tag { INTRA_SUBMAP, INTER_SUBMAP } tag;
  };
  //Landmark 相当于路标，每个 Node 与 Landmark 之间的相对位姿也应该加入到约束之中。
  struct LandmarkNode {
    struct LandmarkObservation {
      int trajectory_id;
      common::Time time;
      transform::Rigid3d landmark_to_tracking_transform;
      double translation_weight;
      double rotation_weight;
    };
    std::vector<LandmarkObservation> landmark_observations;
    absl::optional<transform::Rigid3d> global_landmark_pose;
    bool frozen = false;
  };

  struct SubmapPose {
    // ? version版本表示什么
    int version;
    transform::Rigid3d pose;
  };

  struct SubmapData {
    //栅格概率图数据
    std::shared_ptr<const Submap> submap;
    //submap 的绝对位姿
    transform::Rigid3d pose;
  };

  struct TrajectoryData {
    double gravity_constant = 9.8;
    std::array<double, 4> imu_calibration{{1., 0., 0., 0.}};
    absl::optional<transform::Rigid3d> fixed_frame_origin_in_map;
  };
  //轨迹的状态
  enum class TrajectoryState { ACTIVE, FINISHED, FROZEN, DELETED };
  //满足一定条件则会调用该回调函数进行优化。可以看到，该函数传入的参数是一系列的 submap 的 id
  //和 Node 的 id。
  using GlobalSlamOptimizationCallback =
      std::function<void(const std::map<int /* trajectory_id */, SubmapId>&,
                         const std::map<int /* trajectory_id */, NodeId>&)>;

  PoseGraphInterface() {}
  virtual ~PoseGraphInterface() {}

  PoseGraphInterface(const PoseGraphInterface&) = delete;
  PoseGraphInterface& operator=(const PoseGraphInterface&) = delete;

  // Waits for all computations to finish and computes optimized poses.
  //最后的全局优化
  virtual void RunFinalOptimization() = 0;

  // Returns data for all submaps.
  // 返回所有 Submap 的数据。
  virtual MapById<SubmapId, SubmapData> GetAllSubmapData() const = 0;

  // Returns the global poses for all submaps.
  // 返回所有 submap 的 pose
  virtual MapById<SubmapId, SubmapPose> GetAllSubmapPoses() const = 0;

  // Returns the transform converting data in the local map frame (i.e. the
  // continuous, non-loop-closed frame) into the global map frame (i.e. the
  // discontinuous, loop-closed frame).
  // 获取由局部坐标系到世界坐标系的变换矩阵
  virtual transform::Rigid3d GetLocalToGlobalTransform(
      int trajectory_id) const = 0;

  // Returns the current optimized trajectories.
  // 返回当前经过优化后的 trajectory 上的所有 Node。这些 Node 构成了这条 trajectory
  virtual MapById<NodeId, TrajectoryNode> GetTrajectoryNodes() const = 0;

  // Returns the current optimized trajectory poses.
  // 返回当前经过优化后的所有的节点 Pose
  virtual MapById<NodeId, TrajectoryNodePose> GetTrajectoryNodePoses()
      const = 0;

  // Returns the states of trajectories.
  virtual std::map<int, TrajectoryState> GetTrajectoryStates() const = 0;

  // Returns the current optimized landmark poses.
  // 返回 Landmark 的 Pose
  virtual std::map<std::string, transform::Rigid3d> GetLandmarkPoses()
      const = 0;

  // Sets global pose of landmark 'landmark_id' to given 'global_pose'.
  // 设置某个 LandMark 的 Pose
  virtual void SetLandmarkPose(const std::string& landmark_id,
                               const transform::Rigid3d& global_pose,
                               const bool frozen = false) = 0;

  // Deletes a trajectory asynchronously.
  virtual void DeleteTrajectory(int trajectory_id) = 0;

  // Checks if the given trajectory is finished.
  // 判断一条 trajectory 是否被 finished
  virtual bool IsTrajectoryFinished(int trajectory_id) const = 0;

  // Checks if the given trajectory is frozen.
  virtual bool IsTrajectoryFrozen(int trajectory_id) const = 0;

  // Returns the trajectory data.
  // 返回 TrajectoryData
  virtual std::map<int, TrajectoryData> GetTrajectoryData() const = 0;

  // Returns the collection of constraints.
  // 返回所有的约束
  virtual std::vector<Constraint> constraints() const = 0;

  // Serializes the constraints and trajectories. If
  // 'include_unfinished_submaps' is set to 'true', unfinished submaps, i.e.
  // submaps that have not yet received all rangefinder data insertions, will
  // be included, otherwise not.
  // 序列化约束和 trajectory
  virtual proto::PoseGraph ToProto(bool include_unfinished_submaps) const = 0;

  // Sets the callback function that is invoked whenever the global optimization
  // problem is solved.
  // 设置回调函数
  virtual void SetGlobalSlamOptimizationCallback(
      GlobalSlamOptimizationCallback callback) = 0;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_GRAPH_INTERFACE_H_
