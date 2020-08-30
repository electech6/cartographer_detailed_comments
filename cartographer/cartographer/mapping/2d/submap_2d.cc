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

#include "cartographer/mapping/2d/submap_2d.h"

#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>

#include "Eigen/Geometry"
#include "absl/memory/memory.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/2d/tsdf_range_data_inserter_2d.h"
#include "cartographer/mapping/range_data_inserter_interface.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

proto::SubmapsOptions2D CreateSubmapsOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::SubmapsOptions2D options;
  options.set_num_range_data(
      parameter_dictionary->GetNonNegativeInt("num_range_data"));
  *options.mutable_grid_options_2d() = CreateGridOptions2D(
      parameter_dictionary->GetDictionary("grid_options_2d").get());
  *options.mutable_range_data_inserter_options() =
      CreateRangeDataInserterOptions(
          parameter_dictionary->GetDictionary("range_data_inserter").get());

  bool valid_range_data_inserter_grid_combination = false;
  const proto::GridOptions2D_GridType& grid_type =
      options.grid_options_2d().grid_type();
  const proto::RangeDataInserterOptions_RangeDataInserterType&
      range_data_inserter_type =
          options.range_data_inserter_options().range_data_inserter_type();
  if (grid_type == proto::GridOptions2D::PROBABILITY_GRID &&
      range_data_inserter_type ==
          proto::RangeDataInserterOptions::PROBABILITY_GRID_INSERTER_2D) {
    valid_range_data_inserter_grid_combination = true;
  }
  if (grid_type == proto::GridOptions2D::TSDF &&
      range_data_inserter_type ==
          proto::RangeDataInserterOptions::TSDF_INSERTER_2D) {
    valid_range_data_inserter_grid_combination = true;
  }
  CHECK(valid_range_data_inserter_grid_combination)
      << "Invalid combination grid_type " << grid_type
      << " with range_data_inserter_type " << range_data_inserter_type;
  CHECK_GT(options.num_range_data(), 0);
  return options;
}
//Submap 的坐标系旋转角度设置为 0.而原点由参数 origin 给出
Submap2D::Submap2D(const Eigen::Vector2f& origin, std::unique_ptr<Grid2D> grid,
                   ValueConversionTables* conversion_tables)
    : Submap(transform::Rigid3d::Translation(
          Eigen::Vector3d(origin.x(), origin.y(), 0.))),
      conversion_tables_(conversion_tables) {
  grid_ = std::move(grid);
}
/**
 * @brief 从 proto 流中构建 Submap2D
 * @param[in] proto 
 * @param[in] conversion_tables 
 */
Submap2D::Submap2D(const proto::Submap2D& proto,
                   ValueConversionTables* conversion_tables)
    : Submap(transform::ToRigid3(proto.local_pose())),
      conversion_tables_(conversion_tables) {
  if (proto.has_grid()) {
    if (proto.grid().has_probability_grid_2d()) {
      grid_ =
          absl::make_unique<ProbabilityGrid>(proto.grid(), conversion_tables_);
    } else if (proto.grid().has_tsdf_2d()) {
      grid_ = absl::make_unique<TSDF2D>(proto.grid(), conversion_tables_);
    } else {
      LOG(FATAL) << "proto::Submap2D has grid with unknown type.";
    }
  }
  set_num_range_data(proto.num_range_data());
  set_insertion_finished(proto.finished());
}
/**
 * @brief 与 proto 流相关的处理：序列化存到proto中
 * @param[in] include_grid_data 
 * @return proto::Submap 
 */
proto::Submap Submap2D::ToProto(const bool include_grid_data) const {
  proto::Submap proto;
  auto* const submap_2d = proto.mutable_submap_2d();
  *submap_2d->mutable_local_pose() = transform::ToProto(local_pose());
  submap_2d->set_num_range_data(num_range_data());
  submap_2d->set_finished(insertion_finished());
  if (include_grid_data) {
    CHECK(grid_);
    ////调用 grid_中的 ToProto 函数把概率图保存到proto 中
    *submap_2d->mutable_grid() = grid_->ToProto();
  }
  return proto;
}
/**
 * @brief 从 proto 流中获取 Submap2D
 * @param[in] proto 
 */
void Submap2D::UpdateFromProto(const proto::Submap& proto) {
  CHECK(proto.has_submap_2d());
  const auto& submap_2d = proto.submap_2d();
  set_num_range_data(submap_2d.num_range_data());
  set_insertion_finished(submap_2d.finished());
  if (proto.submap_2d().has_grid()) {
    if (proto.submap_2d().grid().has_probability_grid_2d()) {
      grid_ = absl::make_unique<ProbabilityGrid>(proto.submap_2d().grid(),
                                                 conversion_tables_);
    } else if (proto.submap_2d().grid().has_tsdf_2d()) {
      grid_ = absl::make_unique<TSDF2D>(proto.submap_2d().grid(),
                                        conversion_tables_);
    } else {
      LOG(FATAL) << "proto::Submap2D has grid with unknown type.";
    }
  }
}
/**
 * @brief 放到 response 中
 * @param[in] response 
 */
void Submap2D::ToResponseProto(
    const transform::Rigid3d&,
    proto::SubmapQuery::Response* const response) const {
  if (!grid_) return;
  response->set_submap_version(num_range_data());
  proto::SubmapQuery::Response::SubmapTexture* const texture =
      response->add_textures();
  grid()->DrawToSubmapTexture(texture, local_pose());
}
/**
 * @brief 插入点云数据
 * @param[in] range_data 
 * @param[in] range_data_inserter 
 */
void Submap2D::InsertRangeData(
    const sensor::RangeData& range_data,
    const RangeDataInserterInterface* range_data_inserter) {
  //检查Grid不为空，且完成标志为未完成
  CHECK(grid_);
  CHECK(!insertion_finished());
  //用概率网格范围数据插入器2D进行插入激光数据,调用 RangeDataInserterInterface 来更新概率图。
  range_data_inserter->Insert(range_data, grid_.get());
  //插入点云数据+1 
  set_num_range_data(num_range_data() + 1);
}

void Submap2D::Finish() {
  //检查Grid不为空，且完成标志为未完成
  CHECK(grid_);
  CHECK(!insertion_finished());
  //计算裁剪栅格地图 
  grid_ = grid_->ComputeCroppedGrid();
  //将插入子图完成设置成true
  set_insertion_finished(true);
}

ActiveSubmaps2D::ActiveSubmaps2D(const proto::SubmapsOptions2D& options)
    : options_(options), range_data_inserter_(CreateRangeDataInserter()) {}

std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::submaps() const {
  return std::vector<std::shared_ptr<const Submap2D>>(submaps_.begin(),
                                                      submaps_.end());
}
/**
 * @brief 向子图中插入点云函数
 * @param[in] range_data 
 * @return std::vector<std::shared_ptr<const Submap2D>> 
 */
std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::InsertRangeData(
    const sensor::RangeData& range_data) {
  //submaps_ 的vector 保留了两个子图，一个用于匹配，一个用于构建。
  //当submaps_ vector 为空或者是submaps_ vector 的最后submap插入的激光数据达到 
  //设定的值options_.num_range_data 时，添加子图AddSubmap
  if (submaps_.empty() ||
      submaps_.back()->num_range_data() == options_.num_range_data()) {
    AddSubmap(range_data.origin.head<2>());
  }
  //给submaps_的vector 中的submap 添加激光数据 ，函数InsertRangeData
  for (auto& submap : submaps_) {
    submap->InsertRangeData(range_data, range_data_inserter_.get());
  }
  //当submaps_ 的vector 的第一个submap 插入激光数据为  2*options_.num_range_data 时，子图完成Finish
  // ? num_range_data指的是点云数量？重叠50%
  if (submaps_.front()->num_range_data() == 2 * options_.num_range_data()) {
    submaps_.front()->Finish();
  }
  return submaps();
}

std::unique_ptr<RangeDataInserterInterface>
ActiveSubmaps2D::CreateRangeDataInserter() {
  switch (options_.range_data_inserter_options().range_data_inserter_type()) {
    case proto::RangeDataInserterOptions::PROBABILITY_GRID_INSERTER_2D:
      return absl::make_unique<ProbabilityGridRangeDataInserter2D>(
          options_.range_data_inserter_options()
              .probability_grid_range_data_inserter_options_2d());
    case proto::RangeDataInserterOptions::TSDF_INSERTER_2D:
      return absl::make_unique<TSDFRangeDataInserter2D>(
          options_.range_data_inserter_options()
              .tsdf_range_data_inserter_options_2d());
    default:
      LOG(FATAL) << "Unknown RangeDataInserterType.";
  }
}
/**
 * @brief CreateGrid函数
 * @param[in] origin 
 * @return std::unique_ptr<GridInterface> 
 */
std::unique_ptr<GridInterface> ActiveSubmaps2D::CreateGrid(
    const Eigen::Vector2f& origin) {
  //初始化栅格地图的大小100 ，获取分辨率的值
  constexpr int kInitialSubmapSize = 100;
  float resolution = options_.grid_options_2d().resolution();
  switch (options_.grid_options_2d().grid_type()) {
    //选择创建栅格地图
    case proto::GridOptions2D::PROBABILITY_GRID:
    //二维栅格地图的构建  Eigen::Vector2d::Ones() 二维列向量[1,1]
      return absl::make_unique<ProbabilityGrid>(
          MapLimits(resolution,
                    origin.cast<double>() + 0.5 * kInitialSubmapSize *
                                                resolution *
                                                Eigen::Vector2d::Ones(),
                    CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
          &conversion_tables_);
    //选择TSDF类型建图，精度更高
    case proto::GridOptions2D::TSDF:
      return absl::make_unique<TSDF2D>(
          MapLimits(resolution,
                    origin.cast<double>() + 0.5 * kInitialSubmapSize *
                                                resolution *
                                                Eigen::Vector2d::Ones(),
                    CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
          options_.range_data_inserter_options()
              .tsdf_range_data_inserter_options_2d()
              .truncation_distance(),
          options_.range_data_inserter_options()
              .tsdf_range_data_inserter_options_2d()
              .maximum_weight(),
          &conversion_tables_);
    default:
      LOG(FATAL) << "Unknown GridType.";
  }
}
/**
 * @brief 添加子图函数
 * @param[in] origin 
 */
void ActiveSubmaps2D::AddSubmap(const Eigen::Vector2f& origin) {
  //检查submaps_ vector 子图数量，大于2个时，删除开始的子图
  if (submaps_.size() >= 2) {
    // This will crop the finished Submap before inserting a new Submap to
    // reduce peak memory usage a bit.
    CHECK(submaps_.front()->insertion_finished());
    submaps_.erase(submaps_.begin());
  }
  //压入子图
  submaps_.push_back(absl::make_unique<Submap2D>(
      origin,
      std::unique_ptr<Grid2D>(
          static_cast<Grid2D*>(CreateGrid(origin).release())),
      &conversion_tables_));
}

}  // namespace mapping
}  // namespace cartographer
