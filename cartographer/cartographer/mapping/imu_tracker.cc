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

#include "cartographer/mapping/imu_tracker.h"

#include <cmath>
#include <limits>

#include "cartographer/common/math.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
/**
 * @brief Construct a new Imu Tracker:: Imu Tracker object构造函数进行初始化
 * 初始化的变量：重力方向、时间、最近线加速度时间
 * @param[in] imu_gravity_time_constant 
 * @param[in] time 
 */
ImuTracker::ImuTracker(const double imu_gravity_time_constant,
                       const common::Time time)
    : imu_gravity_time_constant_(imu_gravity_time_constant),
      time_(time),
      last_linear_acceleration_time_(common::Time::min()),
      orientation_(Eigen::Quaterniond::Identity()),//初始方向角为单位矩阵
      gravity_vector_(Eigen::Vector3d::UnitZ()),//重力方向初始化为[0,0,1]
      //imu角速度初始化为0
      imu_angular_velocity_(Eigen::Vector3d::Zero()) {}
/**
 * @brief 更新imu的状态
 * @param[in] time 
 */
void ImuTracker::Advance(const common::Time time) {
  CHECK_LE(time_, time);
  //计算时间段=time-time_
  const double delta_t = common::ToSeconds(time - time_);
  //角速度乘以时间，然后转化成 RotationnQuaternion,这是这段时间的姿态变化量
  const Eigen::Quaterniond rotation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(imu_angular_velocity_ * delta_t));
  //以当前姿态 orientation_为基准，再乘以姿态变化量。得到最新的姿态
  orientation_ = (orientation_ * rotation).normalized();
  //更新重力方向
  gravity_vector_ = rotation.conjugate() * gravity_vector_;
  //更新时间
  time_ = time;
}
/**
 * @brief 根据读数更新线加速度。这里的线加速度是经过重力校正的。
 * @param[in] imu_linear_acceleration 
 */
 // Step 1 求时间差delta_t=time_-last_time_
 // Step 2 alpha=1-e^(delta_t/g)
 // Step 3 gravity_vector_=(1-alpha)*gravity_vector_+alpha*imu_linear_acceleration
 // Step 4 更新方向角
void ImuTracker::AddImuLinearAccelerationObservation(
    const Eigen::Vector3d& imu_linear_acceleration) {
  // Update the 'gravity_vector_' with an exponential moving average using the
  // 'imu_gravity_time_constant'.
  const double delta_t =
      last_linear_acceleration_time_ > common::Time::min()
          ? common::ToSeconds(time_ - last_linear_acceleration_time_)
          : std::numeric_limits<double>::infinity();//无限表示
  //更新时间
  last_linear_acceleration_time_ = time_;
  //指数来确定权重，因为有噪声的存在，时间差越大，当前的权重越大
  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);
  gravity_vector_ =
      (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
  // Change the 'orientation_' so that it agrees with the current
  // 'gravity_vector_'.
  // ? 求重力的旋转矩阵
  const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
      gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());
  //更新方向角
  orientation_ = (orientation_ * rotation).normalized();
  CHECK_GT((orientation_ * gravity_vector_).z(), 0.);
  CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
}
/**
 * @brief  根据读数更新角速度
 * @param[in] imu_angular_velocity 
 */
void ImuTracker::AddImuAngularVelocityObservation(
    const Eigen::Vector3d& imu_angular_velocity) {
  imu_angular_velocity_ = imu_angular_velocity;
}

}  // namespace mapping
}  // namespace cartographer
