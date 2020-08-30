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

#include "cartographer/mapping/pose_extrapolator.h"

#include <algorithm>

#include "absl/memory/memory.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
/**
 * @brief Construct a new Pose Extrapolator:: Pose Extrapolator object起到一个初始化的作用
 * @param[in] pose_queue_duration 
 * @param[in] imu_gravity_time_constant 
 */
PoseExtrapolator::PoseExtrapolator(const common::Duration pose_queue_duration,
                                   double imu_gravity_time_constant)
    : pose_queue_duration_(pose_queue_duration),
      gravity_time_constant_(imu_gravity_time_constant),
      // 初始化为单位矩阵
      cached_extrapolated_pose_{common::Time::min(),
                                transform::Rigid3d::Identity()} {}
/**
 * @brief 根据 IMU 数据来初始化一个 PoseExtrapolator
 * @param[in] pose_queue_duration 
 * @param[in] imu_gravity_time_constant 
 * @param[in] imu_data 
 * @return std::unique_ptr<PoseExtrapolator> 
 */
std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
    const common::Duration pose_queue_duration,
    const double imu_gravity_time_constant, const sensor::ImuData& imu_data) {
  //创建一个PoseExtrapolator类型的指针
  auto extrapolator = absl::make_unique<PoseExtrapolator>(
      pose_queue_duration, imu_gravity_time_constant);
  //加入IMU数据
  extrapolator->AddImuData(imu_data);
  //创建ImuTracker类型的指针，并更新
  extrapolator->imu_tracker_ =
      absl::make_unique<ImuTracker>(imu_gravity_time_constant, imu_data.time);
  extrapolator->imu_tracker_->AddImuLinearAccelerationObservation(
      imu_data.linear_acceleration);
  extrapolator->imu_tracker_->AddImuAngularVelocityObservation(
      imu_data.angular_velocity);
  extrapolator->imu_tracker_->Advance(imu_data.time);
  //添加位姿，注意此时的位姿是从 Imu 测量得到的。
  extrapolator->AddPose(
      imu_data.time,
      transform::Rigid3d::Rotation(extrapolator->imu_tracker_->orientation()));
  return extrapolator;
}
/**
 * @brief 得到最新位姿时间
 * @return common::Time 
 */
common::Time PoseExtrapolator::GetLastPoseTime() const {
  if (timed_pose_queue_.empty()) {
    return common::Time::min();
  }
  return timed_pose_queue_.back().time;
}
/**
 * @brief 返回解算结果的最新时间
 * @return common::Time 
 */
common::Time PoseExtrapolator::GetLastExtrapolatedTime() const {
  if (!extrapolation_imu_tracker_) {
    return common::Time::min();
  }
  return extrapolation_imu_tracker_->time();
}
/**
 * @brief 在时刻 time 往 Pose 队列中添加一个 Pose
 * @param[in] time 
 * @param[in] pose 
 */
void PoseExtrapolator::AddPose(const common::Time time,
                               const transform::Rigid3d& pose) {
  // 如果 ImuTracker 还没有建立，则需要建立一个 ImuTracker                             
  if (imu_tracker_ == nullptr) {
    common::Time tracker_start = time;
    // 如果 IMU 数据队列不为空，则以当前时间和 IMU 数据中的最早时刻的较小值为初始时刻建立一个 ImuTracker
    if (!imu_data_.empty()) {
      tracker_start = std::min(tracker_start, imu_data_.front().time);
    }
    //给imu_tracker地址
    imu_tracker_ =
        absl::make_unique<ImuTracker>(gravity_time_constant_, tracker_start);
  }
  //压入队列
  timed_pose_queue_.push_back(TimedPose{time, pose});
  //如果队列队列大于 2，并且时间间隔已经大于我们设定的 pose_queue_duration_时，则把队列之前的元素删除
  while (timed_pose_queue_.size() > 2 &&
         timed_pose_queue_[1].time <= time - pose_queue_duration_) {
    timed_pose_queue_.pop_front();
  }
  //更新最新估计的速度
  UpdateVelocitiesFromPoses();
  //更新 ImuTracker
  AdvanceImuTracker(time, imu_tracker_.get());
  //更新 IMU 数据队列
  TrimImuData();
  //更新里程计数据队列
  TrimOdometryData();
  // 里程计和融合结果都以当前 IMU 的 tracker 为准。
  odometry_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
  extrapolation_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
}
/**
 * @brief 把新的 IMU 数据添加到队列中，删去队列中的过期数据
 * @param[in] imu_data 
 */
void PoseExtrapolator::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(timed_pose_queue_.empty() ||
        imu_data.time >= timed_pose_queue_.back().time);
  imu_data_.push_back(imu_data);
  TrimImuData();
}
/**
 * @brief 把新的odometry数据添加到队列中，删去队列中的过期数据
 * @param[in] odometry_data 
 */
void PoseExtrapolator::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  //里程计数据时间是否晚于Pose队列的最新时间
  CHECK(timed_pose_queue_.empty() ||
        odometry_data.time >= timed_pose_queue_.back().time);
  //压入里程计数据队列
  odometry_data_.push_back(odometry_data);
  //删去队列中的过期数据
  TrimOdometryData();
  if (odometry_data_.size() < 2) {
    return;
  }
  //如果里程计数据大于2了，则我们可以根据里程计数据进行一下状态估计
  // TODO(whess): Improve by using more than just the last two odometry poses.
  // Compute extrapolation in the tracking frame.
  //赋值里程计的时间差
  const sensor::OdometryData& odometry_data_oldest = odometry_data_.front();
  const sensor::OdometryData& odometry_data_newest = odometry_data_.back();
  //计算时间差
  const double odometry_time_delta =
      common::ToSeconds(odometry_data_oldest.time - odometry_data_newest.time);
  //计算里程计位姿差
  const transform::Rigid3d odometry_pose_delta =
      odometry_data_newest.pose.inverse() * odometry_data_oldest.pose;
  //计算里程计角速度
  angular_velocity_from_odometry_ =
      transform::RotationQuaternionToAngleAxisVector(
          odometry_pose_delta.rotation()) /
      odometry_time_delta;
  if (timed_pose_queue_.empty()) {
    return;
  }
  //从里程计数据中估计线速度.
  const Eigen::Vector3d
      linear_velocity_in_tracking_frame_at_newest_odometry_time =
          odometry_pose_delta.translation() / odometry_time_delta;
  //计算里程计坐标系与基准坐标系的变化姿态
  const Eigen::Quaterniond orientation_at_newest_odometry_time =
      timed_pose_queue_.back().pose.rotation() *
      ExtrapolateRotation(odometry_data_newest.time,
                          odometry_imu_tracker_.get());
  //换算成基准坐标系下的线速度
  linear_velocity_from_odometry_ =
      orientation_at_newest_odometry_time *
      linear_velocity_in_tracking_frame_at_newest_odometry_time;
}
/**
 * @brief 对位姿进行插值,得到time时刻的位姿.
 * @param[in] time 
 * @return transform::Rigid3d 
 */
transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const common::Time time) {
  //位姿队列中最新的队列.
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  //必须要比newest_timed_pose的时间大.
  CHECK_GE(time, newest_timed_pose.time);
  if (cached_extrapolated_pose_.time != time) {
    //估计平移值
    const Eigen::Vector3d translation =
        ExtrapolateTranslation(time) + newest_timed_pose.pose.translation();
    //估计旋转值
    const Eigen::Quaterniond rotation =
        newest_timed_pose.pose.rotation() *
        ExtrapolateRotation(time, extrapolation_imu_tracker_.get());
    cached_extrapolated_pose_ =
        TimedPose{time, transform::Rigid3d{translation, rotation}};
  }
  return cached_extrapolated_pose_.pose;
}
/**
 * @brief 计算重力向量,用imu-tracker来进行计算.
 * @param[in] time 
 * @return Eigen::Quaterniond 
 */
Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
    const common::Time time) {
  ImuTracker imu_tracker = *imu_tracker_;
  AdvanceImuTracker(time, &imu_tracker);
  return imu_tracker.orientation();
}
/**
 * @brief 从一个 Pose 队列中估计机器人的线速度和角速度
 * 取出 timed_pose_queue_这个队列中最早和最新的两个 Pose 做差，然后除以时间得到机器人的速度。
 */
void PoseExtrapolator::UpdateVelocitiesFromPoses() {
  // 判断这个 Pose 队列的长度，如果小于 2 则没法进行估计
  if (timed_pose_queue_.size() < 2) {
    // We need two poses to estimate velocities.
    return;
  }
  //检查队列是否为空
  CHECK(!timed_pose_queue_.empty());
  // 取出队列最末尾的一个 Pose,也就是最新时间点的 Pose,并记录相应的时间
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const auto newest_time = newest_timed_pose.time;
  // 取出队列最开头的一个 Pose，也就是最旧时间点的 Pose,并记录相应的时间
  const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
  const auto oldest_time = oldest_timed_pose.time;
  // 两者的时间差
  const double queue_delta = common::ToSeconds(newest_time - oldest_time);
  // 如果时间差小于pose_queue_duration_，则估计不准，弹出警告信息
  if (queue_delta < common::ToSeconds(pose_queue_duration_)) {
    LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                 << queue_delta << " s";
    return;
  }
  //获取两个时刻各自的Pose
  const transform::Rigid3d& newest_pose = newest_timed_pose.pose;
  const transform::Rigid3d& oldest_pose = oldest_timed_pose.pose;
  //线速度即为两个 Pose 的 translation 部分相减后除以间隔时间
  linear_velocity_from_poses_ =
      (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
  //角速度是两个 Pose 的 rotation 部分的差除以间隔时间
  angular_velocity_from_poses_ =
      transform::RotationQuaternionToAngleAxisVector(
          oldest_pose.rotation().inverse() * newest_pose.rotation()) /
      queue_delta;
}
//删去队列中无用的IMU数据
void PoseExtrapolator::TrimImuData() {
  while (imu_data_.size() > 1 && !timed_pose_queue_.empty() &&
         imu_data_[1].time <= timed_pose_queue_.back().time) {
    imu_data_.pop_front();
  }
}
//删去队列中无用的里程计数据
// ? 为什么上面大于1，而这大于2
void PoseExtrapolator::TrimOdometryData() {
  while (odometry_data_.size() > 2 && !timed_pose_queue_.empty() &&
         odometry_data_[1].time <= timed_pose_queue_.back().time) {
    odometry_data_.pop_front();
  }
}
/**
 * @brief 提取IMU数据队列中的数据，更新ImuTracker
 * @param[in] time 
 * @param[in] imu_tracker 
 */
void PoseExtrapolator::AdvanceImuTracker(const common::Time time,
                                         ImuTracker* const imu_tracker) const {
  CHECK_GE(time, imu_tracker->time());
  //如果 IMU 数据队列为空，或当前时间要比 IMU 数据队列中最早期的时间还要早。说明没有可用的IMU 数据
  //这时候根据时间推算。对于角速度，是用从 Pose 中估计的角速度或从里程计获得的角速度更新ImuTracker 的角速度
  if (imu_data_.empty() || time < imu_data_.front().time) {
    // There is no IMU data until 'time', so we advance the ImuTracker and use
    // the angular velocities from poses and fake gravity to help 2D stability.
    imu_tracker->Advance(time);
    imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());
    // ! 角速度的求法
    imu_tracker->AddImuAngularVelocityObservation(
        odometry_data_.size() < 2 ? angular_velocity_from_poses_
                                  : angular_velocity_from_odometry_);
    return;
  }
  // 如果 ImuTracker 维护的时间早于 IMU 数据队列最早期的时间
  if (imu_tracker->time() < imu_data_.front().time) {
    // Advance to the beginning of 'imu_data_'.
    // 先把 ImuTracker 更新到 IMU 数据来临的那一刻
    imu_tracker->Advance(imu_data_.front().time);
  }
  //然后依次取出 IMU 数据队列中的数据，更新 ImuTracker，直到 IMU 数据的时间比指定时间 time要早。
  auto it = std::lower_bound(
      imu_data_.begin(), imu_data_.end(), imu_tracker->time(),
      [](const sensor::ImuData& imu_data, const common::Time& time) {
        return imu_data.time < time;
      });
  while (it != imu_data_.end() && it->time < time) {
    imu_tracker->Advance(it->time);
    imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration);
    imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);
    ++it;
  }
  imu_tracker->Advance(time);
}
/**
 * @brief 计算旋转矩阵的增长量
 * @param[in] time 
 * @param[in] imu_tracker 
 * @return Eigen::Quaterniond 
 */
Eigen::Quaterniond PoseExtrapolator::ExtrapolateRotation(
    const common::Time time, ImuTracker* const imu_tracker) const {
  //检查指定时间是否大于等于 ImuTracker 的时间。
  CHECK_GE(time, imu_tracker->time());
  //更新ImuTracker到指定的time
  AdvanceImuTracker(time, imu_tracker);
  //最新时刻的姿态
  const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();
  //求取姿态变化量：最新时刻姿态的逆乘以当前的姿态
  return last_orientation.inverse() * imu_tracker->orientation();
}
/**
 * @brief 计算位移的增长量
 * @param[in] time 
 * @return Eigen::Vector3d 
 */
Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(common::Time time) {
  //获取Pose队列中最新的时刻
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  //算时间差
  const double extrapolation_delta =
      common::ToSeconds(time - newest_timed_pose.time);
  //如果没有里程计数据，则把从 Pose 队列中估计的线速度乘以时间
  if (odometry_data_.size() < 2) {
    return extrapolation_delta * linear_velocity_from_poses_;
  }
  // 如果有里程计数据，则更信任里程计速度，直接把从里程计处获得的线速度乘以时间
  return extrapolation_delta * linear_velocity_from_odometry_;
}

}  // namespace mapping
}  // namespace cartographer
