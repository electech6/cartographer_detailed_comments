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

#include "cartographer/mapping/internal/optimization/ceres_pose.h"

namespace cartographer {
namespace mapping {
namespace optimization {

///将位姿转化为结构体
CeresPose::Data FromPose(const transform::Rigid3d& pose) {
    ///std::array的构造方式和数组相同
  return CeresPose::Data{{{pose.translation().x(), pose.translation().y(),
                           pose.translation().z()}},
                         {{pose.rotation().w(), pose.rotation().x(),
                           pose.rotation().y(), pose.rotation().z()}}};
}

/// 获取位姿 向ceres问题中添加顶点
/// 初始位姿估计 平移全局局部维度转换类 旋转全局局部维度转换类 ceres问题
CeresPose::CeresPose(
    const transform::Rigid3d& pose,
    std::unique_ptr<ceres::LocalParameterization> translation_parametrization,
    std::unique_ptr<ceres::LocalParameterization> rotation_parametrization,
    ceres::Problem* problem)
    : data_(std::make_shared<CeresPose::Data>(FromPose(pose))) {
    /// (unique_ptr).release() 调用release 会切断unique_ptr 和它原来管理的对象的联系。
    /// release 返回的指针通常被用来初始化另一个智能指针或给另一个智能指针赋值。
  problem->AddParameterBlock(data_->translation.data(), 3,
                             translation_parametrization.release());
  problem->AddParameterBlock(data_->rotation.data(), 4,
                             rotation_parametrization.release());
}

///将data_中的数据转化为Rigid位姿
const transform::Rigid3d CeresPose::ToRigid() const {
  return transform::Rigid3d::FromArrays(data_->rotation, data_->translation);
}

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer
