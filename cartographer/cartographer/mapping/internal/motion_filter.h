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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_MOTION_FILTER_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_MOTION_FILTER_H_

#include <limits>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/motion_filter_options.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {
// 从 Proto 流中读取配置文件，设置配置项，主要包括最大时间间隔、最大角度间隔、最大距离间隔等
proto::MotionFilterOptions CreateMotionFilterOptions(
    common::LuaParameterDictionary* parameter_dictionary);

// Takes poses as input and filters them to get fewer poses.
class MotionFilter {
 public:
  explicit MotionFilter(const proto::MotionFilterOptions& options);

  // If the accumulated motion (linear, rotational, or time) is above the
  // threshold, returns false. Otherwise the relative motion is accumulated and
  // true is returned.
  // 根据预先设置的阈值，如果累积运动超过提前预设的阈值，则返回 false；
  // 否则返回 true，然后把该数据累加上
  bool IsSimilar(common::Time time, const transform::Rigid3d& pose);

 private:
 //配置项，主要包括最大时间间隔、最大角度间隔、最大距离间隔等
  const proto::MotionFilterOptions options_;
  //总共的 pose 数
  int num_total_ = 0;
  //过滤之后剩下的 pose 数
  int num_different_ = 0;
  //上一次时间
  common::Time last_time_;
  //上一次的 pose
  transform::Rigid3d last_pose_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_MOTION_FILTER_H_
