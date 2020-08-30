/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/mapping/internal/range_data_collator.h"

#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

///如果该传感器点云数据存在 则该点云结构体时间作为当前结束时间 取出各传感器点云中的点
///时间戳在当前起始时间到结束时间间的点云 并剔除所有传感器点云中的点 时间戳在结束时间之前的
///如果该传感器点云数据不存在 且希望该传感器点云数据存在 取其他传感器点云中最老时间作为当前结束时间
///输入: 传感器id 带时间的点云数据
///输出: 各传感器时间戳在当前起始终止时间段内的点云的点
sensor::TimedPointCloudOriginData RangeDataCollator::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& timed_point_cloud_data) {
  CHECK_NE(expected_sensor_ids_.count(sensor_id), 0);
  // TODO(gaschler): These two cases can probably be one.
  ///使用count，返回的是被查找元素的个数。如果有，返回1；否则，返回0。
  ///该传感器存在
  if (id_to_pending_data_.count(sensor_id) != 0) {
    current_start_ = current_end_;
    // When we have two messages of the same sensor, move forward the older of
    // the two (do not send out current).
    current_end_ = id_to_pending_data_.at(sensor_id).time; ///该传感器点云最新时间
    /// 取出各传感器点云中的点 时间戳在当前起始时间到结束时间间的点云
    /// 并剔除所有传感器点云中的点 时间戳在结束时间之前的
    auto result = CropAndMerge();
    id_to_pending_data_.emplace(sensor_id, timed_point_cloud_data); ///添加最新数据
    return result;
  }
  ///如果不存在该传感器 添加
  id_to_pending_data_.emplace(sensor_id, timed_point_cloud_data);
  ///不期望该传感器存在
  if (expected_sensor_ids_.size() != id_to_pending_data_.size()) {
    return {};
  }
  current_start_ = current_end_;
  // We have messages from all sensors, move forward to oldest.
  common::Time oldest_timestamp = common::Time::max(); ///所有传感器点云时间中最老的时间
  for (const auto& pair : id_to_pending_data_) {
    oldest_timestamp = std::min(oldest_timestamp, pair.second.time);
  }
  current_end_ = oldest_timestamp;
  return CropAndMerge();
}

/// 取出各传感器点云中的点 时间戳在当前起始时间到结束时间间的点云
/// 并剔除所有传感器点云中的点 时间戳在结束时间之前的
sensor::TimedPointCloudOriginData RangeDataCollator::CropAndMerge() {
    ///所有传感器点云中的点 时间在当前起始时间到终止时间中的
  sensor::TimedPointCloudOriginData result{current_end_, {}, {}};
  bool warned_for_dropped_points = false;
  ///每一个传感器点云
  for (auto it = id_to_pending_data_.begin();
       it != id_to_pending_data_.end();) {
    sensor::TimedPointCloudData& data = it->second;
    sensor::TimedPointCloud& ranges = it->second.ranges;

    ///找到 开始时间<点云中的点时间+点云时间<=结束时间 的所有点
    auto overlap_begin = ranges.begin(); ///时间范围内的起始
    ///点云时间+点时间<当前起始时间的所有点
    while (overlap_begin < ranges.end() &&
           data.time + common::FromSeconds((*overlap_begin).time) <
               current_start_) {
      ++overlap_begin;
    }
    ///点云时间+点时间<=当前结束时间的所有点
    auto overlap_end = overlap_begin;
    while (overlap_end < ranges.end() &&
           data.time + common::FromSeconds((*overlap_end).time) <=
               current_end_) {
      ++overlap_end;
    }
    ///丢弃点云vector中时间比起始时间早的点云
    if (ranges.begin() < overlap_begin && !warned_for_dropped_points) {
      LOG(WARNING) << "Dropped " << std::distance(ranges.begin(), overlap_begin)
                   << " earlier points.";
      warned_for_dropped_points = true;
    }

    // Copy overlapping range.
    ///如果存在点
    if (overlap_begin < overlap_end) {
      std::size_t origin_index = result.origins.size(); ///原始点云vector大小
      result.origins.push_back(data.origin);
      const float time_correction =
          static_cast<float>(common::ToSeconds(data.time - current_end_));
      for (auto overlap_it = overlap_begin; overlap_it != overlap_end;
           ++overlap_it) {
        sensor::TimedPointCloudOriginData::RangeMeasurement point{*overlap_it,
                                                                  origin_index};
        // current_end_ + point_time[3]_after == in_timestamp +
        // point_time[3]_before
        point.point_time.time += time_correction;
        result.ranges.push_back(point);
      }
    }

    // Drop buffered points until overlap_end.
    ///剔除所点云中的点 时间戳在结束时间之前的
    ///该传感器点云中所有的点 时间早于 当前结束时间
    if (overlap_end == ranges.end()) {
      it = id_to_pending_data_.erase(it); ///清空点云缓存
    } else if (overlap_end == ranges.begin()) {
      ++it;
    ///保留该传感器点云中时间大于当前结束时间的点
    } else {
      data = sensor::TimedPointCloudData{
          data.time, data.origin,
          sensor::TimedPointCloud(overlap_end, ranges.end())};
      ++it;
    }
  }

  ///std::sort(起始,结束,比较函数) 按照比较函数返回true的方式排序
  ///对各传感器点云中的点按时间从小到大排序
  std::sort(result.ranges.begin(), result.ranges.end(),
            [](const sensor::TimedPointCloudOriginData::RangeMeasurement& a,
               const sensor::TimedPointCloudOriginData::RangeMeasurement& b) {
              return a.point_time.time < b.point_time.time;
            });
  return result;
}

}  // namespace mapping
}  // namespace cartographer
