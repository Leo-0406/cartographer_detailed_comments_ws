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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_COLLATED_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_COLLATED_TRAJECTORY_BUILDER_H_

#include <chrono>
#include <map>
#include <memory>
#include <set>
#include <string>

#include "cartographer/common/internal/rate_timer.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/internal/local_slam_result_data.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/collator_interface.h"
#include "cartographer/sensor/internal/dispatchable.h"

namespace cartographer {
namespace mapping {

// Collates sensor data using a sensor::CollatorInterface, then passes it on to
// a mapping::TrajectoryBuilderInterface which is common for 2D and 3D.
// 使用 sensor::CollatorInterface 整理传感器数据, 进行数据分发
// 然后将其传递到2D和3D通用的 mapping::TrajectoryBuilderInterface

// 处理传感器数据, 使其按照时间排列, 然后传入GlobalTrajectoryBuilder
class CollatedTrajectoryBuilder : public TrajectoryBuilderInterface {
 public:
  using SensorId = TrajectoryBuilderInterface::SensorId;

  CollatedTrajectoryBuilder(
      const proto::TrajectoryBuilderOptions& trajectory_options,
      sensor::CollatorInterface* sensor_collator, int trajectory_id,
      const std::set<SensorId>& expected_sensor_ids,
      std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder);
  // 虚析构函数， override 重写基类函数 ，参数列表，返回值类型，const使用，与基类保持一致 
  ~CollatedTrajectoryBuilder() override {}

  CollatedTrajectoryBuilder(const CollatedTrajectoryBuilder&) = delete;
  CollatedTrajectoryBuilder& operator=(const CollatedTrajectoryBuilder&) =
      delete;

  // 处理雷达点云数据
  void AddSensorData(
      const std::string& sensor_id,   // 接收一个传感器话题
      const sensor::TimedPointCloudData& timed_point_cloud_data) override {
    AddData(sensor::MakeDispatchable(sensor_id, timed_point_cloud_data));
  }

  // 处理IMU数据
  void AddSensorData(const std::string& sensor_id,
                     const sensor::ImuData& imu_data) override {
    AddData(sensor::MakeDispatchable(sensor_id, imu_data));
  }

  // 处理里程计数据
  void AddSensorData(const std::string& sensor_id,
                     const sensor::OdometryData& odometry_data) override {
    AddData(sensor::MakeDispatchable(sensor_id, odometry_data));
  }

  // 根据参数决定gps数据是否需要排序
  // AddData与wrapped_trajectory_builder_->AddSensorData只能选一种
  // 因为AddData最终调用的就是wrapped_trajectory_builder_->AddSensorData
  void AddSensorData(
      const std::string& sensor_id,     // gps数据
      const sensor::FixedFramePoseData& fixed_frame_pose_data) override {
    if (collate_fixed_frame_) { // collate_fixed_frame_在trajectory_builde.lua中定义，默认为true
      AddData(sensor::MakeDispatchable(sensor_id, fixed_frame_pose_data));
      return;
    }
    wrapped_trajectory_builder_->AddSensorData(sensor_id,
                                               fixed_frame_pose_data);
  }

  // 根据参数决定Landmark数据是否需要排序
  void AddSensorData(const std::string& sensor_id,
                     const sensor::LandmarkData& landmark_data) override {
    if (collate_landmarks_) {   // landmark   同上
      AddData(sensor::MakeDispatchable(sensor_id, landmark_data));
      return;
    }
    wrapped_trajectory_builder_->AddSensorData(sensor_id, landmark_data);
  }

  // 将local slam 的结果也作为一种传感器数据进行处理
  void AddLocalSlamResultData(std::unique_ptr<mapping::LocalSlamResultData>
                                  local_slam_result_data) override {
    AddData(std::move(local_slam_result_data));
  }

 private:
  void AddData(std::unique_ptr<sensor::Data> data);

  void HandleCollatedSensorData(const std::string& sensor_id,
                                std::unique_ptr<sensor::Data> data);

  sensor::CollatorInterface* const sensor_collator_;
  const bool collate_landmarks_;
  const bool collate_fixed_frame_;
  const int trajectory_id_;
  std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder_;

  // Time at which we last logged the rates of incoming sensor data.
  std::chrono::steady_clock::time_point last_logging_time_;
  std::map<std::string, common::RateTimer<>> rate_timers_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_COLLATED_TRAJECTORY_BUILDER_H_
