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

#include "cartographer/mapping/internal/3d/scan_matching/fast_correlative_scan_matcher_3d.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/3d/scan_matching/low_resolution_matcher.h"
#include "cartographer/mapping/proto/scan_matching//fast_correlative_scan_matcher_options_3d.pb.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

proto::FastCorrelativeScanMatcherOptions3D
CreateFastCorrelativeScanMatcherOptions3D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::FastCorrelativeScanMatcherOptions3D options;
  ///使用几层深度 一般是8
  options.set_branch_and_bound_depth(
      parameter_dictionary->GetInt("branch_and_bound_depth"));
  ///保证原始分辨率的层数 一般是3
  options.set_full_resolution_depth(
      parameter_dictionary->GetInt("full_resolution_depth"));
  ///两个点云直方图匹配要求的最小得分
  options.set_min_rotational_score(
      parameter_dictionary->GetDouble("min_rotational_score"));
  options.set_min_low_resolution_score(
      parameter_dictionary->GetDouble("min_low_resolution_score"));
  ///搜索窗口大小
  options.set_linear_xy_search_window(
      parameter_dictionary->GetDouble("linear_xy_search_window"));
  options.set_linear_z_search_window(
      parameter_dictionary->GetDouble("linear_z_search_window"));
  options.set_angular_search_window(
      parameter_dictionary->GetDouble("angular_search_window"));
  return options;
}

/*
 * 这里使用的降采样网格,其占用的储存空间依然和原始网格相同,所以构造使用的分辨率是一样的.
 * 只是在某一深度depth下,边长为2^depth储存单元的正方形储存空间内,只有一个有0-255数据
 * 且该数据储存在上一网格中视为(0,0,0)的位置,其他七个位置没有数据 迭代器在迭代的时候会自动跳过
 * 如果设置网格降采样 则需要把3D序号除以2
 */

///输入: 概率格栅地图 配置
///将概率格栅地图转化为0-255得分地图
PrecomputationGridStack3D::PrecomputationGridStack3D(
    const HybridGrid& hybrid_grid,
    const proto::FastCorrelativeScanMatcherOptions3D& options) {
  CHECK_GE(options.branch_and_bound_depth(), 1);
  CHECK_GE(options.full_resolution_depth(), 1);
  ///0-255离散地图列表的第一个地图是概率地图的0-255地图
  precomputation_grids_.reserve(options.branch_and_bound_depth());
  precomputation_grids_.push_back(ConvertToPrecomputationGrid(hybrid_grid));
  Eigen::Array3i last_width = Eigen::Array3i::Ones();
  for (int depth = 1; depth != options.branch_and_bound_depth(); ++depth) {
    const bool half_resolution = depth >= options.full_resolution_depth(); ///是否降低分辨率
    ///depth等于3 next_width = (8,8,8)网格
    const Eigen::Array3i next_width = ((1 << depth) * Eigen::Array3i::Ones()); ///该深度网格大小
    const int full_voxels_per_high_resolution_voxel = ///每个网格的分辨率
        1 << std::max(0, depth - options.full_resolution_depth());
    ///根据深度和分辨率 计算当前网格(0,0,0)-(1,1,1)网格的位移大小
    const Eigen::Array3i shift = (next_width - last_width +
                                  (full_voxels_per_high_resolution_voxel - 1)) /
                                 full_voxels_per_high_resolution_voxel;
    precomputation_grids_.push_back(
        PrecomputeGrid(precomputation_grids_.back(), half_resolution, shift));
    last_width = next_width;
  }
}

///离散后的点云
struct DiscreteScan3D {
  transform::Rigid3f pose; ///当前帧相对上一帧可能的位姿变换
  // Contains a vector of discretized scans for each 'depth'.
  std::vector<std::vector<Eigen::Array3i>> cell_indices_per_depth; ///点云中的点在每一个深度0-255地图中的3D序号
  float rotational_score; ///旋转得分
};

///候选位姿
struct Candidate3D {
  Candidate3D(const int scan_index, const Eigen::Array3i& offset)
      : scan_index(scan_index), offset(offset) {}

  static Candidate3D Unsuccessful() {
    return Candidate3D(0, Eigen::Array3i::Zero());
  }

  // Index into the discrete scans vectors.
  int scan_index; ///直方图匹配成功的离散点云 一维序号 表示第几个旋转

  // Linear offset from the initial pose in cell indices. For lower resolution
  // candidates this is the lowest offset of the 2^depth x 2^depth x 2^depth
  // block of possibilities.
  //单元格索引中与初始姿势的线性偏移。对于低分辨率候选，这是2^depth x 2^depth x 2^depth块可能性的最小偏移量。
  Eigen::Array3i offset; ///线性搜索偏移

  // Score, higher is better.
  // 返回特殊值“正无穷大”，由浮点类型T表示。
  // 仅当std：：numeric_limits<T>：：has_infinity==true时才有意义。
  float score = -std::numeric_limits<float>::infinity();

  // Score of the low resolution matcher.
  float low_resolution_score = 0.f; ///该候选位姿在低分辨率和局部地图的匹配得分

  bool operator<(const Candidate3D& other) const { return score < other.score; }
  bool operator>(const Candidate3D& other) const { return score > other.score; }
};

///格栅 低分辨率格栅 旋转扫描匹配直方图 配置
FastCorrelativeScanMatcher3D::FastCorrelativeScanMatcher3D(
    const HybridGrid& hybrid_grid,
    const HybridGrid* const low_resolution_hybrid_grid,
    const Eigen::VectorXf* rotational_scan_matcher_histogram,
    const proto::FastCorrelativeScanMatcherOptions3D& options)
    : options_(options),
      resolution_(hybrid_grid.resolution()),
      width_in_voxels_(hybrid_grid.grid_size()),
      precomputation_grid_stack_(
          absl::make_unique<PrecomputationGridStack3D>(hybrid_grid, options)), //用hybrid_grid, options构造的指针
      low_resolution_hybrid_grid_(low_resolution_hybrid_grid),
      rotational_scan_matcher_(rotational_scan_matcher_histogram) {}

FastCorrelativeScanMatcher3D::~FastCorrelativeScanMatcher3D() {}

///上一帧位姿 局部地图位姿 常数 局部定位最小得分
std::unique_ptr<FastCorrelativeScanMatcher3D::Result>
FastCorrelativeScanMatcher3D::Match(
    const transform::Rigid3d& global_node_pose,
    const transform::Rigid3d& global_submap_pose,
    const TrajectoryNode::Data& constant_data, const float min_score) const {
    ///low_resolution_matcher是形参是pose的函数
    ///该函数: 使用位姿变换点云中的点 并对在低分辨率栅格地图下所有有概率的点求和 作为低分辨率匹配得分
  const auto low_resolution_matcher = scan_matching::CreateLowResolutionMatcher(
      low_resolution_hybrid_grid_, &constant_data.low_resolution_point_cloud);
  const SearchParameters search_parameters{
      common::RoundToInt(options_.linear_xy_search_window() / resolution_),
      common::RoundToInt(options_.linear_z_search_window() / resolution_),
      options_.angular_search_window(), &low_resolution_matcher};
  ///.cast<flaot>() 强制类型转换
  return MatchWithSearchParameters(
      search_parameters, global_node_pose.cast<float>(),
      global_submap_pose.cast<float>(),
      constant_data.high_resolution_point_cloud,
      constant_data.rotational_scan_matcher_histogram,
      constant_data.gravity_alignment, min_score);
}

std::unique_ptr<FastCorrelativeScanMatcher3D::Result>
FastCorrelativeScanMatcher3D::MatchFullSubmap(
    const Eigen::Quaterniond& global_node_rotation,
    const Eigen::Quaterniond& global_submap_rotation,
    const TrajectoryNode::Data& constant_data, const float min_score) const {
  float max_point_distance = 0.f;
  for (const sensor::RangefinderPoint& point :
       constant_data.high_resolution_point_cloud) {
    max_point_distance = std::max(max_point_distance, point.position.norm());
  }
  const int linear_window_size =
      (width_in_voxels_ + 1) / 2 +
      common::RoundToInt(max_point_distance / resolution_ + 0.5f);
  const auto low_resolution_matcher = scan_matching::CreateLowResolutionMatcher(
      low_resolution_hybrid_grid_, &constant_data.low_resolution_point_cloud);
  const SearchParameters search_parameters{
      linear_window_size, linear_window_size, M_PI, &low_resolution_matcher};
  return MatchWithSearchParameters(
      search_parameters,
      transform::Rigid3f::Rotation(global_node_rotation.cast<float>()),
      transform::Rigid3f::Rotation(global_submap_rotation.cast<float>()),
      constant_data.high_resolution_point_cloud,
      constant_data.rotational_scan_matcher_histogram,
      constant_data.gravity_alignment, min_score);
}

/////搜索参数 上一帧位姿 局部地图位姿 高分辨率点云 旋转直方图 重力方向 局部定位最小得分
std::unique_ptr<FastCorrelativeScanMatcher3D::Result>
FastCorrelativeScanMatcher3D::MatchWithSearchParameters(
    const FastCorrelativeScanMatcher3D::SearchParameters& search_parameters,
    const transform::Rigid3f& global_node_pose,
    const transform::Rigid3f& global_submap_pose,
    const sensor::PointCloud& point_cloud,
    const Eigen::VectorXf& rotational_scan_matcher_histogram,
    const Eigen::Quaterniond& gravity_alignment, const float min_score) const {
  const std::vector<DiscreteScan3D> discrete_scans = GenerateDiscreteScans(
    ///找到直方图匹配度高的 旋转位姿变换
    ///每一个位姿构造一个DiscreteScan3D类
      search_parameters, point_cloud, rotational_scan_matcher_histogram,
      gravity_alignment, global_node_pose, global_submap_pose);

  ///找到低分辨率下的候选位姿 并排序
  const std::vector<Candidate3D> lowest_resolution_candidates =
      ComputeLowestResolutionCandidates(search_parameters, discrete_scans);

  ///最优候选位姿
  const Candidate3D best_candidate = BranchAndBound(
      search_parameters, discrete_scans, lowest_resolution_candidates,
      precomputation_grid_stack_->max_depth(), min_score);
  if (best_candidate.score > min_score) {
    return absl::make_unique<Result>(Result{
        best_candidate.score,
        GetPoseFromCandidate(discrete_scans, best_candidate).cast<double>(),
        discrete_scans[best_candidate.scan_index].rotational_score,
        best_candidate.low_resolution_score});
  }
  return nullptr;
}

///搜索参数 当前帧点云 当前帧相对上一帧可能的位姿变换 得分
///求出变换后点云在0-255地图中每个深度的位置序号 并构造一个DiscreteScan3D类
DiscreteScan3D FastCorrelativeScanMatcher3D::DiscretizeScan(
    const FastCorrelativeScanMatcher3D::SearchParameters& search_parameters,
    const sensor::PointCloud& point_cloud, const transform::Rigid3f& pose,
    const float rotational_score) const {
  std::vector<std::vector<Eigen::Array3i>> cell_indices_per_depth; ///变换后点云每个深度0-255地图的3D序号
  const PrecomputationGrid3D& original_grid = ///原始局部地图
      precomputation_grid_stack_->Get(0);
  std::vector<Eigen::Array3i> full_resolution_cell_indices;
  ///对于转换后的点云的点 将坐标转化为离散序号(有正有负)
  for (const sensor::RangefinderPoint& point :
       sensor::TransformPointCloud(point_cloud, pose)) {
    full_resolution_cell_indices.push_back(
        original_grid.GetCellIndex(point.position));
  }
  const int full_resolution_depth = std::min(options_.full_resolution_depth(), ///保持分辨率的深度
                                             options_.branch_and_bound_depth());
  CHECK_GE(full_resolution_depth, 1);
  ///分辨率不变 点云坐标不变
  for (int i = 0; i != full_resolution_depth; ++i) {
    cell_indices_per_depth.push_back(full_resolution_cell_indices);
  }
  ///降级分辨率的深度层数
  const int low_resolution_depth =
      options_.branch_and_bound_depth() - full_resolution_depth;
  CHECK_GE(low_resolution_depth, 0);
  ///线性搜索窗口 单方向
  const Eigen::Array3i search_window_start(
      -search_parameters.linear_xy_window_size,
      -search_parameters.linear_xy_window_size,
      -search_parameters.linear_z_window_size);
  ///对于每一个低分辨率深度
  for (int i = 0; i != low_resolution_depth; ++i) {
    const int reduction_exponent = i + 1;
    const Eigen::Array3i low_resolution_search_window_start( ///缩放后搜索窗口(单方向)
        search_window_start[0] >> reduction_exponent,
        search_window_start[1] >> reduction_exponent,
        search_window_start[2] >> reduction_exponent);
    cell_indices_per_depth.emplace_back();
    ///每一个转换后点云中的点在原始格栅的地图中的3D序号
    for (const Eigen::Array3i& cell_index : full_resolution_cell_indices) {
      const Eigen::Array3i cell_at_start = cell_index + search_window_start; ///把所有的点云变成只有负的坐标
      ///缩放点云坐标
      const Eigen::Array3i low_resolution_cell_at_start(
          cell_at_start[0] >> reduction_exponent,
          cell_at_start[1] >> reduction_exponent,
          cell_at_start[2] >> reduction_exponent);
      ///将点云变成正负都有 添加到vector(变换后点云每个深度0-255地图的3D序号)
      cell_indices_per_depth.back().push_back(
          low_resolution_cell_at_start - low_resolution_search_window_start);
    }
  }
  ///构造离散后的点云
  return DiscreteScan3D{pose, cell_indices_per_depth, rotational_score};
}

///先使用可能的角度匹配 当前点云和局部地图匹直方图
///再求直方图匹配得分较高的点云 其变换点云在原始格栅的地图中每个深度的位置序号
///每一个位姿构造一个DiscreteScan3D类
///输入: 搜索参数 点云 旋转直方图 重力方向 上一帧位姿 局部地图位姿
///输出: 可能匹配的位姿vector<离散后点云类(相对位姿变换,变换后点云在0-255地图中每个深度的位置序号,旋转得分)>
std::vector<DiscreteScan3D> FastCorrelativeScanMatcher3D::GenerateDiscreteScans(
    const FastCorrelativeScanMatcher3D::SearchParameters& search_parameters,
    const sensor::PointCloud& point_cloud,
    const Eigen::VectorXf& rotational_scan_matcher_histogram,
    const Eigen::Quaterniond& gravity_alignment,
    const transform::Rigid3f& global_node_pose,
    const transform::Rigid3f& global_submap_pose) const {
  std::vector<DiscreteScan3D> result; ///可能匹配的位姿vector<离散后点云类>
  // We set this value to something on the order of resolution to make sure that
  // the std::acos() below is defined.
  float max_scan_range = 3.f * resolution_;
  for (const sensor::RangefinderPoint& point : point_cloud) {
    const float range = point.position.norm();
    max_scan_range = std::max(range, max_scan_range);
  }
  const float kSafetyMargin = 1.f - 1e-2f;
  const float angular_step_size = ///角度搜索步长
      kSafetyMargin * std::acos(1.f - common::Pow2(resolution_) /
                                          (2.f * common::Pow2(max_scan_range)));
  const int angular_window_size = common::RoundToInt( ///同一个方向的角度搜索次数
      search_parameters.angular_search_window / angular_step_size);
  std::vector<float> angles; ///角度搜索列表
  for (int rz = -angular_window_size; rz <= angular_window_size; ++rz) {
    angles.push_back(rz * angular_step_size);
  }
  ///局部地图和上一帧之间的位姿变换
  const transform::Rigid3f node_to_submap =
      global_submap_pose.inverse() * global_node_pose;
  ///尝试匹配所有角度的当前帧直方图和局部地图的直方图
  ///搜索角度直方图向量和局部地图直方图向量的夹角cos值 vector
  const std::vector<float> scores = rotational_scan_matcher_.Match(
      rotational_scan_matcher_histogram,
      transform::GetYaw(node_to_submap.rotation() *
                        gravity_alignment.inverse().cast<float>()),
      angles);
  for (size_t i = 0; i != angles.size(); ++i) {
      ///抛弃得分小于最低要求的得分
    if (scores[i] < options_.min_rotational_score()) {
      continue;
    }
    ///计算直方图匹配成功的当前帧位姿
    const Eigen::Vector3f angle_axis(0.f, 0.f, angles[i]);
    // It's important to apply the 'angle_axis' rotation between the translation
    // and rotation of the 'initial_pose', so that the rotation is around the
    // origin of the range data, and yaw is in map frame.
    const transform::Rigid3f pose(
        node_to_submap.translation(),
        global_submap_pose.rotation().inverse() *
            transform::AngleAxisVectorToRotationQuaternion(angle_axis) *
            global_node_pose.rotation());
    result.push_back(
        ///求出变换后点云在0-255地图中每个深度的位置序号 并构造一个DiscreteScan3D类
        DiscretizeScan(search_parameters, point_cloud, pose, scores[i]));
  }
  return result;
}

///构造低分辨率候选位姿列表
///输入: 搜索参数 位姿个数
///输出: vector<旋转位姿 最大深度搜索步长>
std::vector<Candidate3D>
FastCorrelativeScanMatcher3D::GenerateLowestResolutionCandidates(
    const FastCorrelativeScanMatcher3D::SearchParameters& search_parameters,
    const int num_discrete_scans) const {
  const int linear_step_size = 1 << precomputation_grid_stack_->max_depth(); ///最大深度线性搜索步长
  ///xyz方向的搜索次数
  ///比如搜索窗是步长 实际上应该搜索 不动 +步长 -步长 三个可能
  const int num_lowest_resolution_linear_xy_candidates =
      (2 * search_parameters.linear_xy_window_size + linear_step_size) /
      linear_step_size;
  const int num_lowest_resolution_linear_z_candidates =
      (2 * search_parameters.linear_z_window_size + linear_step_size) /
      linear_step_size;
  ///总共低分辨率搜索次数
  const int num_candidates =
      num_discrete_scans *
      common::Power(num_lowest_resolution_linear_xy_candidates, 2) *
      num_lowest_resolution_linear_z_candidates;
  std::vector<Candidate3D> candidates;
  candidates.reserve(num_candidates);
  ///每一个候选位姿
  for (int scan_index = 0; scan_index != num_discrete_scans; ++scan_index) {
    for (int z = -search_parameters.linear_z_window_size;
         z <= search_parameters.linear_z_window_size; z += linear_step_size) {
      for (int y = -search_parameters.linear_xy_window_size;
           y <= search_parameters.linear_xy_window_size;
           y += linear_step_size) {
        for (int x = -search_parameters.linear_xy_window_size;
             x <= search_parameters.linear_xy_window_size;
             x += linear_step_size) {
          candidates.emplace_back(scan_index, Eigen::Array3i(x, y, z));
        }
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

///根据分辨率平移对应旋转点云 对所有平移后点云在局部地图中的得分求和
///按照概率从打排序候选位姿
///输入:深度 离散点云列表 候选位姿列表
///返回:根据概率排序后的候选位姿列表
void FastCorrelativeScanMatcher3D::ScoreCandidates(
    const int depth, const std::vector<DiscreteScan3D>& discrete_scans,
    std::vector<Candidate3D>* const candidates) const {
  const int reduction_exponent = ///该深度分辨率
      std::max(0, depth - options_.full_resolution_depth() + 1);
  ///每一个候选位姿
  for (Candidate3D& candidate : *candidates) {
    int sum = 0; ///点云平移后局部地图在点云位置上的得分
    const DiscreteScan3D& discrete_scan = discrete_scans[candidate.scan_index]; ///该位姿对应的旋转后的点云
    ///线性位移乘上缩放系数
    const Eigen::Array3i offset(candidate.offset[0] >> reduction_exponent,
                                candidate.offset[1] >> reduction_exponent,
                                candidate.offset[2] >> reduction_exponent);
    CHECK_LT(depth, discrete_scan.cell_indices_per_depth.size());
    ///该深度下每一个点的3D序号(有正有负 已经缩放)
    for (const Eigen::Array3i& cell_index :
         discrete_scan.cell_indices_per_depth[depth]) {
      const Eigen::Array3i proposed_cell_index = cell_index + offset; ///平移点云
      sum += precomputation_grid_stack_->Get(depth).value(proposed_cell_index);
    }
    ///得分转化为概率
    candidate.score = PrecomputationGrid3D::ToProbability(
        sum /
        static_cast<float>(discrete_scan.cell_indices_per_depth[depth].size()));
  }
  ///从大到小排序
  std::sort(candidates->begin(), candidates->end(),
            std::greater<Candidate3D>());
}

///找到低分辨率下的候选位姿 并排序
///输入:搜索参数 候选位姿DiscreteScan3D
///输出:根据低分辨率下的概率排序后的候选位姿列表
std::vector<Candidate3D>
FastCorrelativeScanMatcher3D::ComputeLowestResolutionCandidates(
    const FastCorrelativeScanMatcher3D::SearchParameters& search_parameters,
    const std::vector<DiscreteScan3D>& discrete_scans) const {
  ///构造低分辨率候选位姿列表
  std::vector<Candidate3D> lowest_resolution_candidates = ///候选位姿列表
      GenerateLowestResolutionCandidates(search_parameters,
                                         discrete_scans.size());
  ///根据分辨率平移对应旋转点云 对所有平移后点云在局部地图中的得分求和
  ///按照概率从打排序候选位姿
  ScoreCandidates(precomputation_grid_stack_->max_depth(), discrete_scans,
                  &lowest_resolution_candidates);
  return lowest_resolution_candidates;
}

///获取候选位姿
transform::Rigid3f FastCorrelativeScanMatcher3D::GetPoseFromCandidate(
    const std::vector<DiscreteScan3D>& discrete_scans,
    const Candidate3D& candidate) const {
  return transform::Rigid3f::Translation(
             resolution_ * candidate.offset.matrix().cast<float>()) *
         discrete_scans[candidate.scan_index].pose;
}

///输入:搜索参数 离散点云 低分辨率候选位姿 最大深度 局部定位最小得分
///输出:最优变换位姿
Candidate3D FastCorrelativeScanMatcher3D::BranchAndBound(
    const FastCorrelativeScanMatcher3D::SearchParameters& search_parameters,
    const std::vector<DiscreteScan3D>& discrete_scans,
    const std::vector<Candidate3D>& candidates, const int candidate_depth,
    float min_score) const {
    ///如果最大深度等于0
  if (candidate_depth == 0) {
      ///每一个低分辨率候选位姿
    for (const Candidate3D& candidate : candidates) {
        ///如果得分较小 直接返回不存在
      if (candidate.score <= min_score) {
        // Return if the candidate is bad because the following candidate will
        // not have better score.
        return Candidate3D::Unsuccessful();
      }
      const float low_resolution_score = ///低分辨率平移后的直方图匹配得分
          (*search_parameters.low_resolution_matcher)(
              GetPoseFromCandidate(discrete_scans, candidate));
      ///如果平移后的直方图匹配的分依旧大于阈值
      if (low_resolution_score >= options_.min_low_resolution_score()) {
        // We found the best candidate that passes the matching function.
        Candidate3D best_candidate = candidate;
        best_candidate.low_resolution_score = low_resolution_score;
        return best_candidate;
      }
    }

    // All candidates have good scores but none passes the matching function.
    return Candidate3D::Unsuccessful();
  }

  ///如果最大深度不等于0
  Candidate3D best_high_resolution_candidate = Candidate3D::Unsuccessful(); ///最好位姿
  best_high_resolution_candidate.score = min_score; ///得分
  ///每一个候选位姿
  for (const Candidate3D& candidate : candidates) {
      ///如果得分小于局部定位最小得分就跳出循环(由于候选位姿是按照得分排序的)
    if (candidate.score <= min_score) {
      break;
    }

    std::vector<Candidate3D> higher_resolution_candidates; ///该深度候选位姿
    const int half_width = 1 << (candidate_depth - 1); ///该深度网格宽度的一半
    ///z的取值只能是 0和half_width
      ///限定该深度搜索范围在搜索窗内
    for (int z : {0, half_width}) {
      if (candidate.offset.z() + z > search_parameters.linear_z_window_size) {
        break;
      }
      for (int y : {0, half_width}) {
        if (candidate.offset.y() + y >
            search_parameters.linear_xy_window_size) {
          break;
        }
        for (int x : {0, half_width}) {
          if (candidate.offset.x() + x >
              search_parameters.linear_xy_window_size) {
            break;
          }
          higher_resolution_candidates.emplace_back(
              candidate.scan_index, candidate.offset + Eigen::Array3i(x, y, z));
        }
      }
    }
    ///取得分满足要求的 进行下一深度搜索
    ScoreCandidates(candidate_depth - 1, discrete_scans,
                    &higher_resolution_candidates);
    ///返回所有深度中得分最多的候选位姿
    best_high_resolution_candidate = std::max(
        best_high_resolution_candidate,
        BranchAndBound(search_parameters, discrete_scans,
                       higher_resolution_candidates, candidate_depth - 1,
                       best_high_resolution_candidate.score));
  }
  return best_high_resolution_candidate;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
