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

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_H_

#include <memory>

#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/sensor/collator_interface.h"

namespace cartographer {
namespace mapping {

proto::MapBuilderOptions CreateMapBuilderOptions(
    common::LuaParameterDictionary *const parameter_dictionary);

// Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
// and a PoseGraph for loop closure.
/**
 * @brief MapBuilder 是对 MapBuilderInterface 的继承和实现， MapBuilder 中的方法都已经在
 * MapBuilderInterface 中定义
 */
class MapBuilder : public MapBuilderInterface {
 public:
  explicit MapBuilder(const proto::MapBuilderOptions &options);
  ~MapBuilder() override {}

  MapBuilder(const MapBuilder &) = delete;
  MapBuilder &operator=(const MapBuilder &) = delete;

  int AddTrajectoryBuilder(
      const std::set<SensorId> &expected_sensor_ids,
      const proto::TrajectoryBuilderOptions &trajectory_options,
      LocalSlamResultCallback local_slam_result_callback) override;

  int AddTrajectoryForDeserialization(
      const proto::TrajectoryBuilderOptionsWithSensorIds
          &options_with_sensor_ids_proto) override;

  void FinishTrajectory(int trajectory_id) override;

  std::string SubmapToProto(const SubmapId &submap_id,
                            proto::SubmapQuery::Response *response) override;

  void SerializeState(bool include_unfinished_submaps,
                      io::ProtoStreamWriterInterface *writer) override;

  bool SerializeStateToFile(bool include_unfinished_submaps,
                            const std::string &filename) override;

  std::map<int, int> LoadState(io::ProtoStreamReaderInterface *reader,
                               bool load_frozen_state) override;

  std::map<int, int> LoadStateFromFile(const std::string &filename,
                                       const bool load_frozen_state) override;

  mapping::PoseGraphInterface *pose_graph() override {
    //unique_ptr 的 get 函数可返回被管理对象地址的指针
    return pose_graph_.get();
  }

  int num_trajectory_builders() const override {
    //向量的 size 即为 TrajectoryBuilder 的数量
    return trajectory_builders_.size();
  }
  /**
   * @brief 如果该 trajectory 没有一个 TrajectoryBuilder，则返回 nullptr' 
   * @param[in] trajectory_id 
   * @return mapping::TrajectoryBuilderInterface* 
   */
  mapping::TrajectoryBuilderInterface *GetTrajectoryBuilder(
      int trajectory_id) const override {
    //根据vector的索引求对应的值地址
    return trajectory_builders_.at(trajectory_id).get();
  }
  /**
   * @brief 获取所有 TrajectoryBuilder 的配置项。
   * @return const std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>& 
   */
  const std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
      &GetAllTrajectoryBuilderOptions() const override {
    return all_trajectory_builder_options_;
  }

 private:
  //MapBuilder 的配置项
  const proto::MapBuilderOptions options_;
  //线程池。个人猜测，应该是为每一条 trajectory 都单独开辟一个线程
  common::ThreadPool thread_pool_;
  //PoseGraph类型的智能指针，该指针用来做 Loop Closure
  std::unique_ptr<PoseGraph> pose_graph_;
  //收集传感器数据的智能指针
  std::unique_ptr<sensor::CollatorInterface> sensor_collator_;
  //一个向量，管理所有的 TrajectoryBuilderInterface;应该是每一个 trajectory 对应了该向量的一个元素
  //每一个 TrajectoryBuilder 对应了机器人运行了一圈。这个向量列表就管理了整个图中的所有 submap
  std::vector<std::unique_ptr<mapping::TrajectoryBuilderInterface>>
      trajectory_builders_;
  //与每个 TrajectoryBuilderInterface 相对应的配置项
  std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
      all_trajectory_builder_options_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
