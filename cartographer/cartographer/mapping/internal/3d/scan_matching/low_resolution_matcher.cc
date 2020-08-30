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

#include "cartographer/mapping/internal/3d/scan_matching/low_resolution_matcher.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

///底分辨率匹配器
///输入: 低分辨率格栅地图 低分辨率点云
///输出: 形参是pose的函数(使用位姿变换点云中的点 并对在低分辨率栅格地图下所有有概率的点求和 作为低分辨率匹配得分)
std::function<float(const transform::Rigid3f&)> CreateLowResolutionMatcher(
    const HybridGrid* low_resolution_grid, const sensor::PointCloud* points) {
    ///匿名函数 隐式安值捕获pose
  return [=](const transform::Rigid3f& pose) {
    float score = 0.f;
    ///使用位姿变换点云中的点 并对在低分辨率栅格地图下所有有概率的点求和 作为低分辨率匹配得分
    for (const sensor::RangefinderPoint& point :
         sensor::TransformPointCloud(*points, pose)) {
      // TODO(zhengj, whess): Interpolate the Grid to get better score.
      score += low_resolution_grid->GetProbability(
          low_resolution_grid->GetCellIndex(point.position));
    }
    return score / points->size();
  };
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
