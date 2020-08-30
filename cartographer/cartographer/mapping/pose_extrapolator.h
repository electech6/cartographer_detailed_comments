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

#ifndef CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
#define CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_

#include <deque>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/imu_tracker.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// Keep poses for a certain duration to estimate linear and angular velocity.
// Uses the velocities to extrapolate motion. Uses IMU and/or odometry data if
// available to improve the extrapolation.
/**
 * @brief 位姿推算器，保存一段时间的位姿来计算线速度和角速度，使用这个速度来估算位姿。
 * Imu和odometry用来改善位姿推算器
 */
class PoseExtrapolator {
 public:
 /**
  * @brief Construct a new Pose Extrapolator object 构造函数
  * @param[in] pose_queue_duration 
  * @param[in] imu_gravity_time_constant 
  */
  explicit PoseExtrapolator(common::Duration pose_queue_duration,
                            double imu_gravity_time_constant);

  PoseExtrapolator(const PoseExtrapolator&) = delete;
  PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;
  /**
   * @brief 
   * @param[in] pose_queue_duration 
   * @param[in] imu_gravity_time_constant 
   * @param[in] imu_data 
   * @return std::unique_ptr<PoseExtrapolator> 
   */
  static std::unique_ptr<PoseExtrapolator> InitializeWithImu(
      common::Duration pose_queue_duration, double imu_gravity_time_constant,
      const sensor::ImuData& imu_data);

  // Returns the time of the last added pose or Time::min() if no pose was added
  // yet.
  /**
   * @brief Get the Last Pose Time object返回最新添加位姿的时间
   * @return common::Time 
   */
  common::Time GetLastPoseTime() const;
  /**
   * @brief Get the Last Extrapolated Time object返回最新位姿推算器时间
   * @return common::Time 
   */
  common::Time GetLastExtrapolatedTime() const;
  /**
   * @brief 添加位姿
   * @param[in] time 
   * @param[in] pose 
   */
  void AddPose(common::Time time, const transform::Rigid3d& pose);
  /**
   * @brief 添加IMU数据
   * @param[in] imu_data 
   */
  void AddImuData(const sensor::ImuData& imu_data);
  /**
   * @brief 添加里程计数据
   * @param[in] odometry_data 
   */
  void AddOdometryData(const sensor::OdometryData& odometry_data);
  /**
   * @brief 得到time時刻的位姿进行外插
   * @param[in] time 
   * @return transform::Rigid3d 
   */
  transform::Rigid3d ExtrapolatePose(common::Time time);

  // Returns the current gravity alignment estimate as a rotation from
  // the tracking frame into a gravity aligned frame.
  /**
   * @brief 评估重力方向
   * @param[in] time 
   * @return Eigen::Quaterniond 
   */
  Eigen::Quaterniond EstimateGravityOrientation(common::Time time);

 private:
  void UpdateVelocitiesFromPoses();
  void TrimImuData();
  void TrimOdometryData();
  void AdvanceImuTracker(common::Time time, ImuTracker* imu_tracker) const;
  Eigen::Quaterniond ExtrapolateRotation(common::Time time,
                                         ImuTracker* imu_tracker) const;
  Eigen::Vector3d ExtrapolateTranslation(common::Time time);

  const common::Duration pose_queue_duration_;
  //时间位姿结构体
  struct TimedPose {
    common::Time time;
    transform::Rigid3d pose;
  };
  // ? 存储要持续跟踪的 Poses,应该是从 ScanMatching 输出的 PoseObservation
  std::deque<TimedPose> timed_pose_queue_;
  // 从持续跟踪一段时间的 Pose 队列中估计出来的线速度和角速度，初始化为 0
  Eigen::Vector3d linear_velocity_from_poses_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_poses_ = Eigen::Vector3d::Zero();

  const double gravity_time_constant_;
  //存储 IMU 数据的队列
  std::deque<sensor::ImuData> imu_data_;
  // ? 三者智能指针的区别
  std::unique_ptr<ImuTracker> imu_tracker_;//存在IMU数据地址
  std::unique_ptr<ImuTracker> odometry_imu_tracker_;//存放由里程计数据的地址
  std::unique_ptr<ImuTracker> extrapolation_imu_tracker_;//存在数据融合后的结果地址
  //缓存一段时间Pose
  TimedPose cached_extrapolated_pose_;
  //里程计数据
  std::deque<sensor::OdometryData> odometry_data_;
  //从odometry中初始化线速度和角速度，初始化为0
  Eigen::Vector3d linear_velocity_from_odometry_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_from_odometry_ = Eigen::Vector3d::Zero();
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_POSE_EXTRAPOLATOR_H_
