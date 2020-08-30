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

#include "cartographer/mapping/internal/3d/scan_matching/precomputation_grid_3d.h"

#include <algorithm>

#include "Eigen/Core"
#include "cartographer/common/math.h"
#include "cartographer/mapping/probability_values.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

// C++11 defines that integer division rounds towards zero. For index math, we
// actually need it to round towards negative infinity. Luckily bit shifts have
// that property.
///将一个数据除以2
inline int DivideByTwoRoundingTowardsNegativeInfinity(const int value) {
  return value >> 1;
}

// Computes the half resolution index corresponding to the full resolution
// 'cell_index'.
///将3D序号每一个维度都除以2
Eigen::Array3i CellIndexAtHalfResolution(const Eigen::Array3i& cell_index) {
  return Eigen::Array3i(
      DivideByTwoRoundingTowardsNegativeInfinity(cell_index[0]),
      DivideByTwoRoundingTowardsNegativeInfinity(cell_index[1]),
      DivideByTwoRoundingTowardsNegativeInfinity(cell_index[2]));
}

}  // namespace

///将概率栅格地图用0-255的离散概率地图表示
PrecomputationGrid3D ConvertToPrecomputationGrid(
    const HybridGrid& hybrid_grid) {
  PrecomputationGrid3D result(hybrid_grid.resolution()); ///使用概率地图构造一个与计算网格
  ///概率地图内的每一个储存单元
  for (auto it = HybridGrid::Iterator(hybrid_grid); !it.Done(); it.Next()) {
    ///把概率栅格中的网格的概率 归一化到0-255之间的整数
    const int cell_value = common::RoundToInt(
        (ValueToProbability(it.GetValue()) - kMinProbability) *
        (255.f / (kMaxProbability - kMinProbability)));
    CHECK_GE(cell_value, 0);
    CHECK_LE(cell_value, 255);
    *result.mutable_value(it.GetCellIndex()) = cell_value; ///修改转化后的网格的值
  }
  return result;
}

/*
 * 这里使用的降采样网格,其占用的储存空间依然和原始网格相同,所以构造使用的分辨率是一样的.
 * 只是在某一深度depth下,边长为2^depth储存单元的正方形储存空间内,只有一个有0-255数据
 * 且该数据储存在上一网格中视为(0,0,0)的位置,其他七个位置没有数据 迭代器在迭代的时候会自动跳过
 * 如果设置网格降采样 则需要把3D序号除以2
 */

///上一深度网格 是否降低分辨率 相对于原始网格的移动
///根据上一深度网格 计算下一深度网格 每个单元的值
PrecomputationGrid3D PrecomputeGrid(const PrecomputationGrid3D& grid,
                                    const bool half_resolution,
                                    const Eigen::Array3i& shift) {
  PrecomputationGrid3D result(grid.resolution()); ///分辨率等于上一深度0-255网格的分辨率
  ///上一深度0-255网格的每一个储存单元
  for (auto it = PrecomputationGrid3D::Iterator(grid); !it.Done(); it.Next()) {
      ///将一个储存单元看做(0,0,0) 将它和周围的(0,0,0),(0,0,1)...(1,1,1) 8个点比较
      ///找到这八个点中最大的 作为该网格的值
      ///如果需要降低分辨率 则将该单元的3D序号每一个维度都除以2
    for (int i = 0; i != 8; ++i) {
      // We use this value to update 8 values in the resulting grid, at
      // position (x - {0, 'shift'}, y - {0, 'shift'}, z - {0, 'shift'}).
      // If 'shift' is 2 ** (depth - 1), where depth 0 is the original grid,
      // this results in precomputation grids analogous to the 2D case.
      const Eigen::Array3i cell_index =
          it.GetCellIndex() - shift * PrecomputationGrid3D::GetOctant(i);
      auto* const cell_value = result.mutable_value(
          half_resolution ? CellIndexAtHalfResolution(cell_index) : cell_index);
      ///一个单元临近的8个单元只保留一个最大值
      *cell_value = std::max(it.GetValue(), *cell_value);
    }
  }
  return result;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
