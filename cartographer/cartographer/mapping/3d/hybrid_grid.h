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

#ifndef CARTOGRAPHER_MAPPING_3D_HYBRID_GRID_H_
#define CARTOGRAPHER_MAPPING_3D_HYBRID_GRID_H_

#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping/proto/3d/hybrid_grid.pb.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// Converts an 'index' with each dimension from 0 to 2^'bits' - 1 to a flat
// z-major index.
///三维坐标 网格大小
///把三维序号变成一维序号
inline int ToFlatIndex(const Eigen::Array3i& index, const int bits) {
  DCHECK((index >= 0).all() && (index < (1 << bits)).all()) << index;
  return (((index.z() << bits) + index.y()) << bits) + index.x();
}

// Converts a flat z-major 'index' to a 3-dimensional index with each dimension
// from 0 to 2^'bits' - 1.
/// 把网格的一维序号转化为3D序号
inline Eigen::Array3i To3DIndex(const int index, const int bits) {
  DCHECK_LT(index, 1 << (3 * bits));

  const int mask = (1 << bits) - 1; //2^bit - 1 == 111....11
  return Eigen::Array3i(index & mask, (index >> bits) & mask,
                        (index >> bits) >> bits);
}

// A function to compare value to the default value. (Allows specializations).
///判断网格中的数据类型是否是默认数据
template <typename TValueType>
bool IsDefaultValue(const TValueType& v) {
  return v == TValueType();
}

// Specialization to compare a std::vector to the default value.
template <typename TElementType>
bool IsDefaultValue(const std::vector<TElementType>& v) {
  return v.empty();
}


//using GridBase = DynamicGrid<NestedGrid<FlatGrid<ValueType, 3>, 3>>

// A flat grid of '2^kBits' x '2^kBits' x '2^kBits' voxels storing values of
// type 'ValueType' in contiguous memory. Indices in each dimension are 0-based.
///三级网格
///2^kBits x 2^kBits x 2^kBits 体素一维网格 在连续内存中存储“ValueType”类型的值。
template <typename TValueType, int kBits>
class FlatGrid {
 public:
  using ValueType = TValueType;

  // Creates a new flat grid with all values being default constructed.
  ///初始化每一个子数据
  FlatGrid() {
    for (ValueType& value : cells_) {
      value = ValueType();
    }
  }

  ///=delete表示禁止使用该函数
  FlatGrid(const FlatGrid&) = delete;
  FlatGrid& operator=(const FlatGrid&) = delete;

  // Returns the number of voxels per dimension.
  ///该一维网格 表示的三维网格 一条边有多少数据单元
  static int grid_size() { return 1 << kBits; }

  // Returns the value stored at 'index', each dimension of 'index' being
  // between 0 and grid_size() - 1.
  ///根据三维坐标返回值
  ValueType value(const Eigen::Array3i& index) const {
    return cells_[ToFlatIndex(index, kBits)];
  }

  // Returns a pointer to a value to allow changing it.
  ///根据三维坐标返回值的引用
  ValueType* mutable_value(const Eigen::Array3i& index) {
    return &cells_[ToFlatIndex(index, kBits)];
  }

  // An iterator for iterating over all values not comparing equal to the
  // default constructed value.
  ///网格迭代器
  class Iterator {
   public:
    Iterator() : current_(nullptr), end_(nullptr) {}

    ///(std::array).data() 一维网格的第一个元素
    ///构造一个一维网格迭代器
    explicit Iterator(const FlatGrid& flat_grid)
        : current_(flat_grid.cells_.data()),
          end_(flat_grid.cells_.data() + flat_grid.cells_.size()) {

        ///迭代器未指向最后一个 网格中的数据类型是默认数据
        ///指向第一个有效元素
      while (!Done() && IsDefaultValue(*current_)) {
        ++current_;
      }
    }

    ///下一个有效数据
    void Next() {
      DCHECK(!Done());
      do {
        ++current_;
      } while (!Done() && IsDefaultValue(*current_));
    }

    bool Done() const { return current_ == end_; }

    ///获取当前指针指向单元的3D序号(只有正)
    Eigen::Array3i GetCellIndex() const {
      DCHECK(!Done());
      const int index = (1 << (3 * kBits)) - (end_ - current_);
      return To3DIndex(index, kBits);
    }

    ///获取当前指针指向单元的值
    const ValueType& GetValue() const {
      DCHECK(!Done());
      return *current_;
    }

   private:
    const ValueType* current_; ///指向一维网格当前元素 当前指针不指向默认数据
    const ValueType* end_; ///指向一维网格最后一个元素
  };

 private:
    ///std::array<类型,个数> 具有固定大小的数组
    ///该网格一共有2^kBits x 2^kBits x 2^kBits个ValueType的数据量
  std::array<ValueType, 1 << (3 * kBits)> cells_; ///该网格所有的数据
};

// A grid consisting of '2^kBits' x '2^kBits' x '2^kBits' grids of type
// 'WrappedGrid'. Wrapped grids are constructed on first access via
// 'mutable_value()'.
///由“2^kBits”x“2^kBits”x“2^kBits”个WrappedGrid网格组成的网格
///由“2^kBits”x“2^kBits”x“2^kBits”个三维网格组成的二级网格
template <typename WrappedGrid, int kBits>
class NestedGrid {
 public:
  using ValueType = typename WrappedGrid::ValueType;

  // Returns the number of voxels per dimension.
  ///返回二级网格一个维度有多少数据单元
  static int grid_size() { return WrappedGrid::grid_size() << kBits; }

  // Returns the value stored at 'index', each dimension of 'index' being
  // between 0 and grid_size() - 1.
  ///根据总网格序号 返回网格值 如果没有值返回数据类型默认值
  ValueType value(const Eigen::Array3i& index) const {
    const Eigen::Array3i meta_index = GetMetaIndex(index); ///二级网格单元3D序号
    const WrappedGrid* const meta_cell =  ///包含该3D序号的二级网格中的三级网格
        meta_cells_[ToFlatIndex(meta_index, kBits)].get();

    if (meta_cell == nullptr) {
      return ValueType();
    }

    const Eigen::Array3i inner_index = ///三级网格3D序号
        index - meta_index * WrappedGrid::grid_size();
    return meta_cell->value(inner_index);
  }

  // Returns a pointer to the value at 'index' to allow changing it. If
  // necessary a new wrapped grid is constructed to contain that value.
  ///根据总网格序号 返回网格值的引用
  ValueType* mutable_value(const Eigen::Array3i& index) {

    const Eigen::Array3i meta_index = GetMetaIndex(index); ///二级网格单元3D序号

    std::unique_ptr<WrappedGrid>& meta_cell =
        meta_cells_[ToFlatIndex(meta_index, kBits)];

    if (meta_cell == nullptr) {
        ///absl谷歌自己的库
      meta_cell = absl::make_unique<WrappedGrid>();
    }

    const Eigen::Array3i inner_index =
        index - meta_index * WrappedGrid::grid_size();

    return meta_cell->mutable_value(inner_index);
  }

  // An iterator for iterating over all values not comparing equal to the
  // default constructed value.
  ///二级网格迭代器
  class Iterator {
   public:
    Iterator() : current_(nullptr), end_(nullptr), nested_iterator_() {}

    explicit Iterator(const NestedGrid& nested_grid)
        : current_(nested_grid.meta_cells_.data()),
          end_(nested_grid.meta_cells_.data() + nested_grid.meta_cells_.size()),
          nested_iterator_() {
      AdvanceToValidNestedIterator();
    }

    void Next() {
      DCHECK(!Done());
      nested_iterator_.Next();
      if (!nested_iterator_.Done()) {
        return;
      }
      ++current_;
      AdvanceToValidNestedIterator();
    }

    bool Done() const { return current_ == end_; }

    ///获取当前迭代器指向的储存单元
    Eigen::Array3i GetCellIndex() const {
      DCHECK(!Done());
      const int index = (1 << (3 * kBits)) - (end_ - current_); ///当前指针指向的三级网格序号
      return To3DIndex(index, kBits) * WrappedGrid::grid_size() +
             nested_iterator_.GetCellIndex();
    }

    ///获取当前迭代器指向的储存单元的引用
    const ValueType& GetValue() const {
      DCHECK(!Done());
      return nested_iterator_.GetValue();
    }

   private:
      ///先移动三级网格迭代器 若三级网格迭代器结束 则将二级网格指针指向下一个有效的三级网格
      ///并构造该三级网格的迭代器
    void AdvanceToValidNestedIterator() {
      for (; !Done(); ++current_) {
        if (*current_ != nullptr) {
          nested_iterator_ = typename WrappedGrid::Iterator(**current_);
          if (!nested_iterator_.Done()) {
            break;
          }
        }
      }
    }

    const std::unique_ptr<WrappedGrid>* current_; ///指向 指向当前三级网格指针 的指针
    const std::unique_ptr<WrappedGrid>* end_; ///指向 指向最后三级网格指针 的指针
    typename WrappedGrid::Iterator nested_iterator_; ///三级网格迭代器
  };

 private:
  // Returns the Eigen::Array3i (meta) index of the meta cell containing
  // 'index'.
  ///把3D坐标转化为二级网格单元3D坐标
  Eigen::Array3i GetMetaIndex(const Eigen::Array3i& index) const {
    DCHECK((index >= 0).all()) << index;
    const Eigen::Array3i meta_index = index / WrappedGrid::grid_size();
    DCHECK((meta_index < (1 << kBits)).all()) << index;
    return meta_index;
  }

  ///一共有2^(3*kBit)个三级网格
  std::array<std::unique_ptr<WrappedGrid>, 1 << (3 * kBits)> meta_cells_; ///指向三级网格的序列
};

// A grid consisting of 2x2x2 grids of type 'WrappedGrid' initially. Wrapped
// grids are constructed on first access via 'mutable_value()'. If necessary,
// the grid grows to twice the size in each dimension. The range of indices is
// (almost) symmetric around the origin, i.e. negative indices are allowed.
// 一个包含2x2x2个WrappedGrid网格的网格
///包含2x2x2个二级网格的一级网格
template <typename WrappedGrid>
class DynamicGrid {
 public:
  using ValueType = typename WrappedGrid::ValueType;

  DynamicGrid() : bits_(1), meta_cells_(8) {}
  DynamicGrid(DynamicGrid&&) = default; ///默认构造函数
  DynamicGrid& operator=(DynamicGrid&&) = default;

  // Returns the current number of voxels per dimension.
  /// 一级网格表示三维空间的边长是 2^(3+3+1)
  ///一级网格的边长
  int grid_size() const { return WrappedGrid::grid_size() << bits_; }

  // Returns the value stored at 'index'.
  ///有正负的总序号 得到网格的值
  ValueType value(const Eigen::Array3i& index) const {
    const Eigen::Array3i shifted_index = index + (grid_size() >> 1); ///把有正有负的序号 变成全是正的总序号
    // The cast to unsigned is for performance to check with 3 comparisons
    // shifted_index.[xyz] >= 0 and shifted_index.[xyz] < grid_size.
    ///判断是否越界
    if ((shifted_index.cast<unsigned int>() >= grid_size()).any()) {
      return ValueType();
    }

    const Eigen::Array3i meta_index = GetMetaIndex(shifted_index); ///一级网格坐标

    const WrappedGrid* const meta_cell = ///包含该3D序号的一级网格中的二级网格
        meta_cells_[ToFlatIndex(meta_index, bits_)].get();

    if (meta_cell == nullptr) {
      return ValueType();
    }

    const Eigen::Array3i inner_index = ///二级网格的序号
        shifted_index - meta_index * WrappedGrid::grid_size();

    return meta_cell->value(inner_index);
  }

  // Returns a pointer to the value at 'index' to allow changing it, dynamically
  // growing the DynamicGrid and constructing new WrappedGrids as needed.
  ///有正负的总序号 得到网格的值的引用
  ValueType* mutable_value(const Eigen::Array3i& index) {
    const Eigen::Array3i shifted_index = index + (grid_size() >> 1); ///把有正有负的序号 变成全是正的总序号
    // The cast to unsigned is for performance to check with 3 comparisons
    // shifted_index.[xyz] >= 0 and shifted_index.[xyz] < grid_size.
      ///判断是否越界
    if ((shifted_index.cast<unsigned int>() >= grid_size()).any()) {
      Grow();
      return mutable_value(index);
    }

    const Eigen::Array3i meta_index = GetMetaIndex(shifted_index); ///一级网格坐标

    std::unique_ptr<WrappedGrid>& meta_cell = ///包含该3D序号的一级网格中的二级网格
        meta_cells_[ToFlatIndex(meta_index, bits_)];

    if (meta_cell == nullptr) {
      meta_cell = absl::make_unique<WrappedGrid>();
    }
    const Eigen::Array3i inner_index = ///二级网格的序号
        shifted_index - meta_index * WrappedGrid::grid_size();
    return meta_cell->mutable_value(inner_index);
  }

  // An iterator for iterating over all values not comparing equal to the
  // default constructed value.
  ///一级网格迭代器
  class Iterator {
   public:
    explicit Iterator(const DynamicGrid& dynamic_grid)
        : bits_(dynamic_grid.bits_),
          current_(dynamic_grid.meta_cells_.data()),
          end_(dynamic_grid.meta_cells_.data() +
               dynamic_grid.meta_cells_.size()),
          nested_iterator_() {

      AdvanceToValidNestedIterator();
    }

    ///下一个子网格
    void Next() {
      DCHECK(!Done());
      nested_iterator_.Next();

      ///如果子网格迭代器未结束
      if (!nested_iterator_.Done()) {
        return;
      }
      ++current_;
        ///将当前指向指向当前子网格指针的指针 指向下一个 子网格迭代器为下一个子网络的迭代器
      AdvanceToValidNestedIterator();
    }

    ///网格指针是否等于最后一个网格指针
    bool Done() const { return current_ == end_; }

      ///获取当前迭代器指向的储存单元的3D序号(有正有负)
    Eigen::Array3i GetCellIndex() const {
      DCHECK(!Done());
      const int outer_index = (1 << (3 * bits_)) - (end_ - current_); ///获取当前指针指向的二级网格序号

      const Eigen::Array3i shifted_index =
          To3DIndex(outer_index, bits_) * WrappedGrid::grid_size() +
          nested_iterator_.GetCellIndex();

      return shifted_index - ((1 << (bits_ - 1)) * WrappedGrid::grid_size());
    }

    const ValueType& GetValue() const {
      DCHECK(!Done());
      return nested_iterator_.GetValue();
    }

    void AdvanceToEnd() { current_ = end_; }

    const std::pair<Eigen::Array3i, ValueType> operator*() const {
      return std::pair<Eigen::Array3i, ValueType>(GetCellIndex(), GetValue());
    }

    Iterator& operator++() {
      Next();
      return *this;
    }

    bool operator!=(const Iterator& it) const {
      return it.current_ != current_;
    }

   private:
      ///先移动二级网格迭代器 若二级网格迭代器结束 则将一级网格指针指向下一个有效的二级网格
      ///并构造该二级网格的迭代器
    void AdvanceToValidNestedIterator() {
      for (; !Done(); ++current_) {
        if (*current_ != nullptr) {
            ///使用有效子网格构造一个迭代器
          nested_iterator_ = typename WrappedGrid::Iterator(**current_);

          ///迭代器未使用过
          if (!nested_iterator_.Done()) {
            break;
          }
        }
      }
    }

    int bits_; ///一条边有2^bits个二级网格
    const std::unique_ptr<WrappedGrid>* current_; ///指向 指向当前二级网格指针 的指针
    const std::unique_ptr<WrappedGrid>* const end_; ///指向 指向最后一二级网格指针 的指针
    typename WrappedGrid::Iterator nested_iterator_; ///二级网络迭代器
  };

 private:
  // Returns the Eigen::Array3i (meta) index of the meta cell containing
  // 'index'.
  ///把3D坐标转化为一级网格单元3D坐标(一共就8个)
  Eigen::Array3i GetMetaIndex(const Eigen::Array3i& index) const {
    DCHECK((index >= 0).all()) << index;
    const Eigen::Array3i meta_index = index / WrappedGrid::grid_size();
    DCHECK((meta_index < (1 << bits_)).all()) << index;
    return meta_index;
  }

  // Grows this grid by a factor of 2 in each of the 3 dimensions.
  ///向外扩张网格 边长扩张一倍
  void Grow() {
    const int new_bits = bits_ + 1;
    CHECK_LE(new_bits, 8);
    std::vector<std::unique_ptr<WrappedGrid>> new_meta_cells_(
        8 * meta_cells_.size());
    for (int z = 0; z != (1 << bits_); ++z) {
      for (int y = 0; y != (1 << bits_); ++y) {
        for (int x = 0; x != (1 << bits_); ++x) {
          const Eigen::Array3i original_meta_index(x, y, z);
          const Eigen::Array3i new_meta_index =
              original_meta_index + (1 << (bits_ - 1));
          new_meta_cells_[ToFlatIndex(new_meta_index, new_bits)] =
              std::move(meta_cells_[ToFlatIndex(original_meta_index, bits_)]);
        }
      }
    }
    meta_cells_ = std::move(new_meta_cells_);
    bits_ = new_bits;
  }

  int bits_; ///一条边有几个二级网格
  std::vector<std::unique_ptr<WrappedGrid>> meta_cells_; ///一级网格中的二级网格指针
};

template <typename ValueType>
using GridBase = DynamicGrid<NestedGrid<FlatGrid<ValueType, 3>, 3>>;

// Represents a 3D grid as a wide, shallow tree.
template <typename ValueType>
class HybridGridBase : public GridBase<ValueType> {
 public:
  using Iterator = typename GridBase<ValueType>::Iterator;

  // Creates a new tree-based probability grid with voxels having edge length
  // 'resolution' around the origin which becomes the center of the cell at
  // index (0, 0, 0).
  explicit HybridGridBase(const float resolution) : resolution_(resolution) {}

  float resolution() const { return resolution_; }

  // Returns the index of the cell containing the 'point'. Indices are integer
  // vectors identifying cells, for this the coordinates are rounded to the next
  // multiple of the resolution.
  /// 将连续点云坐标 转化为离散格栅序号
  Eigen::Array3i GetCellIndex(const Eigen::Vector3f& point) const {
    Eigen::Array3f index = point.array() / resolution_;
    return Eigen::Array3i(common::RoundToInt(index.x()),
                          common::RoundToInt(index.y()),
                          common::RoundToInt(index.z()));
  }

  // Returns one of the octants, (0, 0, 0), (1, 0, 0), ..., (1, 1, 1).
  ///将八进制数转化为2进制
  static Eigen::Array3i GetOctant(const int i) {
    DCHECK_GE(i, 0);
    DCHECK_LT(i, 8);
    return Eigen::Array3i(static_cast<bool>(i & 1), static_cast<bool>(i & 2),
                          static_cast<bool>(i & 4));
  }

  // Returns the center of the cell at 'index'.
  Eigen::Vector3f GetCenterOfCell(const Eigen::Array3i& index) const {
    return index.matrix().cast<float>() * resolution_;
  }

  // Iterator functions for range-based for loops.
  Iterator begin() const { return Iterator(*this); }

  Iterator end() const {
    Iterator it(*this);
    it.AdvanceToEnd();
    return it;
  }

 private:
  // Edge length of each voxel.
  const float resolution_;
};

// A grid containing probability values stored using 15 bits, and an update
// marker per voxel.
// Points are expected to be close to the origin. Points far from the origin
// require the grid to grow dynamically. For centimeter resolution, points
// can only be tens of meters from the origin.
// The hard limit of cell indexes is +/- 8192 around the origin.
// 含使用15位存储的概率值的概率栅格地图
class HybridGrid : public HybridGridBase<uint16> {
 public:
  explicit HybridGrid(const float resolution)
      : HybridGridBase<uint16>(resolution) {}

  explicit HybridGrid(const proto::HybridGrid& proto)
      : HybridGrid(proto.resolution()) {
    CHECK_EQ(proto.values_size(), proto.x_indices_size());
    CHECK_EQ(proto.values_size(), proto.y_indices_size());
    CHECK_EQ(proto.values_size(), proto.z_indices_size());
    for (int i = 0; i < proto.values_size(); ++i) {
      // SetProbability does some error checking for us.
      SetProbability(Eigen::Vector3i(proto.x_indices(i), proto.y_indices(i),
                                     proto.z_indices(i)),
                     ValueToProbability(proto.values(i)));
    }
  }

  // Sets the probability of the cell at 'index' to the given 'probability'.
  void SetProbability(const Eigen::Array3i& index, const float probability) {
    *mutable_value(index) = ProbabilityToValue(probability);
  }

  // Finishes the update sequence.
  void FinishUpdate() {
    while (!update_indices_.empty()) {
      DCHECK_GE(*update_indices_.back(), kUpdateMarker);
      *update_indices_.back() -= kUpdateMarker;
      update_indices_.pop_back();
    }
  }

  // Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
  // to the probability of the cell at 'index' if the cell has not already been
  // updated. Multiple updates of the same cell will be ignored until
  // FinishUpdate() is called. Returns true if the cell was updated.
  //
  // If this is the first call to ApplyOdds() for the specified cell, its value
  // will be set to probability corresponding to 'odds'.
  //根据该点原来的离散值 以及当前观测到的结果 更新地图栅格的可能性
  bool ApplyLookupTable(const Eigen::Array3i& index,
                        const std::vector<uint16>& table) {
    DCHECK_EQ(table.size(), kUpdateMarker);
    uint16* const cell = mutable_value(index); //网格的指针

    //如果栅格越界
    if (*cell >= kUpdateMarker) {
      return false;
    }

    update_indices_.push_back(cell);
    *cell = table[*cell]; //根据该点原来的离散值 以及当前观测到的结果 更新地图栅格的可能性
    DCHECK_GE(*cell, kUpdateMarker);
    return true;
  }

  // Returns the probability of the cell with 'index'.
  float GetProbability(const Eigen::Array3i& index) const {
    return ValueToProbability(value(index));
  }

  // Returns true if the probability at the specified 'index' is known.
  bool IsKnown(const Eigen::Array3i& index) const { return value(index) != 0; }

  proto::HybridGrid ToProto() const {
    CHECK(update_indices_.empty()) << "Serializing a grid during an update is "
                                      "not supported. Finish the update first.";
    proto::HybridGrid result;
    result.set_resolution(resolution());
    for (const auto it : *this) {
      result.add_x_indices(it.first.x());
      result.add_y_indices(it.first.y());
      result.add_z_indices(it.first.z());
      result.add_values(it.second);
    }
    return result;
  }

 private:
  // Markers at changed cells.
  std::vector<ValueType*> update_indices_; ///概率更新的网格单元
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_HYBRID_GRID_H_
