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

#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"

#include <limits>
#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace mapping {

static auto* kLocalSlamLatencyMetric = metrics::Gauge::Null();
static auto* kLocalSlamRealTimeRatio = metrics::Gauge::Null();
static auto* kLocalSlamCpuRealTimeRatio = metrics::Gauge::Null();
static auto* kRealTimeCorrelativeScanMatcherScoreMetric =
    metrics::Histogram::Null();
static auto* kCeresScanMatcherCostMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualDistanceMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualAngleMetric = metrics::Histogram::Null();

/**
 * @brief 构造函数
 * 
 * @param[in] options 
 * @param[in] expected_range_sensor_ids 所有range类型的话题
 */
LocalTrajectoryBuilder2D::LocalTrajectoryBuilder2D(
    const proto::LocalTrajectoryBuilderOptions2D& options,  // protobuf格式的配置文件
    const std::vector<std::string>& expected_range_sensor_ids)
    : options_(options),   // options配置文件来源：trajectory_builder_2d.lua
      active_submaps_(options.submaps_options()),
      motion_filter_(options_.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()),
      range_data_collator_(expected_range_sensor_ids) {}

LocalTrajectoryBuilder2D::~LocalTrajectoryBuilder2D() {}

/**
 * @brief 先进行点云的旋转与z方向的滤波, 然后再进行体素滤波减少数据量
 * 
 * @param[in] transform_to_gravity_aligned_frame 将点云变换到原点处, 且姿态为0的坐标变换
 * @param[in] range_data 传入的点云
 * @return sensor::RangeData 处理后的点云 拷贝
 */
sensor::RangeData
LocalTrajectoryBuilder2D::TransformToGravityAlignedFrameAndFilter(
                              // 传入的坐标变换
    const transform::Rigid3f& transform_to_gravity_aligned_frame,
    const sensor::RangeData& range_data) const {
  // Step: 5 将原点位于机器人当前位姿处的点云 转成 原点位于local坐标系原点处的点云, 再进行z轴上的过滤
  const sensor::RangeData cropped =
      sensor::CropRangeData(sensor::TransformRangeData(//将点云从local_slam坐标系转换到坐标系原点处
                                range_data, transform_to_gravity_aligned_frame),
                            options_.min_z(), options_.max_z()); // param: min_z max_z
  // Step: 6 对点云进行体素滤波
  return sensor::RangeData{
      cropped.origin,
      sensor::VoxelFilter(cropped.returns, options_.voxel_filter_size()), // param: voxel_filter_size
      sensor::VoxelFilter(cropped.misses, options_.voxel_filter_size())};
}

/**
 * @brief 进行扫描匹配
 * 
 * @param[in] time 点云的时间
 * @param[in] pose_prediction 先验位姿
 * @param[in] filtered_gravity_aligned_point_cloud 匹配用的点云
 * @return std::unique_ptr<transform::Rigid2d> 匹配后的二维位姿
 */
std::unique_ptr<transform::Rigid2d> LocalTrajectoryBuilder2D::ScanMatch(
    const common::Time time, const transform::Rigid2d& pose_prediction,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud) {
  if (active_submaps_.submaps().empty()) {  
    // 第1个雷达数据到来时，无地图，无法进行扫描匹配，会直接返回先验位姿，位姿推测器估计而来
    return absl::make_unique<transform::Rigid2d>(pose_prediction);
  }
  // 使用active_submaps_的第一个子图进行匹配
  std::shared_ptr<const Submap2D> matching_submap =
      active_submaps_.submaps().front();
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.
  transform::Rigid2d initial_ceres_pose = pose_prediction;

  // 根据参数决定是否 使用correlative_scan_matching对先验位姿进行校准
  if (options_.use_online_correlative_scan_matching()) {
    const double score = real_time_correlative_scan_matcher_.Match(
        pose_prediction, filtered_gravity_aligned_point_cloud,
        *matching_submap->grid(), &initial_ceres_pose);
    kRealTimeCorrelativeScanMatcherScoreMetric->Observe(score);
  }

  auto pose_observation = absl::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;
  // 使用ceres进行扫描匹配
  ceres_scan_matcher_.Match(pose_prediction.translation(), initial_ceres_pose,
                            filtered_gravity_aligned_point_cloud,
                            *matching_submap->grid(), pose_observation.get(),
                            &summary);
  // 一些度量
  if (pose_observation) {
    kCeresScanMatcherCostMetric->Observe(summary.final_cost);
    const double residual_distance =
        (pose_observation->translation() - pose_prediction.translation())
            .norm();
    kScanMatcherResidualDistanceMetric->Observe(residual_distance);
    const double residual_angle =
        std::abs(pose_observation->rotation().angle() -
                 pose_prediction.rotation().angle());
    kScanMatcherResidualAngleMetric->Observe(residual_angle);
  }
  // 返回ceres计算后的位姿
  return pose_observation;
}

/**
 * @brief 处理点云数据, 进行扫描匹配, 将点云写成地图
 * 
 * @param[in] sensor_id 点云数据对应的话题名称
 * @param[in] unsynchronized_data 传入的点云数据
 * @return std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult> 匹配后的结果
 */
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& unsynchronized_data) {
  
  // Step: 1 进行多个雷达点云数据的时间同步, 点云的坐标是相对于tracking_frame的
  auto synchronized_data =                          // 未同步的点云数据
      range_data_collator_.AddRangeData(sensor_id, unsynchronized_data);
  if (synchronized_data.ranges.empty()) {
    LOG(INFO) << "Range data collator filling buffer.";
    return nullptr;
  }

  const common::Time& time = synchronized_data.time;
  // Initialize extrapolator now if we do not ever use an IMU.
  // 如果不用imu, 就在雷达这初始化位姿推测器
  if (!options_.use_imu_data()) {
    InitializeExtrapolator(time);
  }

  if (extrapolator_ == nullptr) { // 位姿预测器的使用，如果没初始化，就先进行初始化，初始化后就AddPose()
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return nullptr;
  }

  CHECK(!synchronized_data.ranges.empty());
  // TODO(gaschler): Check if this can strictly be 0.
  // 检查点云最后一个点的时间是否小于0
  CHECK_LE(synchronized_data.ranges.back().point_time.time, 0.f);

  // 计算第一个点的时间
  const common::Time time_first_point =
      time +
      common::FromSeconds(synchronized_data.ranges.front().point_time.time);
  // 只有在extrapolator_初始化时, GetLastPoseTime()是common::Time::min()
  if (time_first_point < extrapolator_->GetLastPoseTime()) {
    LOG(INFO) << "Extrapolator is still initializing.";
    return nullptr;
  }
  // 坐标变换
  std::vector<transform::Rigid3f> range_data_poses;
  // 空间申请
  range_data_poses.reserve(synchronized_data.ranges.size());
  bool warned = false;

  // 预测得到每一个时间点的位姿
  for (const auto& range : synchronized_data.ranges) {
    common::Time time_point = time + common::FromSeconds(range.point_time.time);
    // 如果该时间比上次预测位姿的时间还要早,说明这个点的时间戳往回走了, 就报错
    if (time_point < extrapolator_->GetLastExtrapolatedTime()) {
      // 一个循环只报一次错
      if (!warned) {
        LOG(ERROR)
            << "Timestamp of individual range data point jumps backwards from "
            << extrapolator_->GetLastExtrapolatedTime() << " to " << time_point;
        warned = true;
      }
      time_point = extrapolator_->GetLastExtrapolatedTime();
    }
    
    // Step: 2 预测出 每个点的时间戳时刻, tracking frame 在 local slam 坐标系下的位姿
    range_data_poses.push_back(
        extrapolator_->ExtrapolatePose(time_point).cast<float>());
  }
    // num_accumulated_成员变量，初始化的时候为0
  if (num_accumulated_ == 0) {
    // 'accumulated_range_data_.origin' is uninitialized until the last
    // accumulation.
    accumulated_range_data_ = sensor::RangeData{{}, {}, {}};
  }

  // Drop any returns below the minimum range and convert returns beyond the
  // maximum range into misses.
  // 对每个数据点进行处理
  for (size_t i = 0; i < synchronized_data.ranges.size(); ++i) {
    // 获取在tracking frame 下点的坐标
    const sensor::TimedRangefinderPoint& hit =
        synchronized_data.ranges[i].point_time;
    // 将点云的origins坐标转到 local slam 坐标系下（坐标系原点也转到local slam下）
    const Eigen::Vector3f origin_in_local =
        // 坐标变换，根据不同的时间戳预测出来的  step2
        range_data_poses[i] *
        // 通过容器保存某一点对应的坐标系原点，通过索引就能找到该点对应的坐标系原点，
        synchronized_data.origins.at(synchronized_data.ranges[i].origin_index);


    /*
    map_frame ="map"为 cartographer中使用的全局坐标系，也是位姿的父坐标系，应当保持默认。
    而tracking_frame="base_link"，"base_link"意为机器人中心坐标系，这个可以做修改。
        tracking_frame指的是由SLAM算法追踪的ROS坐标系的ID，如果在使用IMU的情况下，也可以（也最好）替换成“imu_link”。
    published_frame="odom"，这个也不固定。这个可以被设置为odom，意为里程计坐标系，也可以被设置为base_link。
        而odom_frame不见得启用，只有在下一个选项provide_odom_frame为true时才启用，这个一般被设为odom，也是里程计的坐标系的名称。
    而provide_odom_frame的作用是，使得坐标系之间经过如下转换以后再发布：published_frame->odom_frame->map_frame。
        如果不是true，则是 published_frame ->map_frame进行发布。
    */
    // Step: 3 运动畸变的去除, 将相对于tracking_frame的hit坐标 转成 local坐标系下的坐标
    sensor::RangefinderPoint hit_in_local =
        // 当前点对应时间的坐标变换
        range_data_poses[i] * sensor::ToRangefinderPoint(hit);
    
    // 计算这个点的距离, 这里用的是去畸变之后的点的距离(运动畸变--->在一帧雷达数据时间段内，距离变化导致）
    // 旋转导致的畸变比较明显，平移相对较小
    const Eigen::Vector3f delta = hit_in_local.position - origin_in_local;
    const float range = delta.norm();// 取模长
    
    // param: min_range max_range
    // trajectory_builde_ed.lua中的 min_range  &&  max_range  (参数，可以调整)
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
        // 在这里可以看到, returns里保存的是local slam下的去畸变之后的点的坐标
        accumulated_range_data_.returns.push_back(hit_in_local);
      } else {
        // Step: 4 超过max_range时的处理: 用一个距离进行替代, 并放入misses里
        hit_in_local.position =
            origin_in_local +
            // param: missing_data_ray_length, 是个比例, 不是距离
            options_.missing_data_ray_length() / range * delta;
                            // range_data.h中定义的结构
        accumulated_range_data_.misses.push_back(hit_in_local);
      }
    }
  } // end for

  // 有一帧有效的数据了
  ++num_accumulated_;

  // param: num_accumulated_range_data 几帧有效的点云数据进行一次扫描匹配，默认值设置为1。
  if (num_accumulated_ >= options_.num_accumulated_range_data()) {
    // 计算2次有效点云数据的的时间差
    const common::Time current_sensor_time = synchronized_data.time;
    absl::optional<common::Duration> sensor_duration;
    if (last_sensor_time_.has_value()) {
      sensor_duration = current_sensor_time - last_sensor_time_.value();
    }
    last_sensor_time_ = current_sensor_time;

    // 重置变量
    num_accumulated_ = 0;

    // 获取机器人当前姿态（相对于启动catrographer的姿态）
    const transform::Rigid3d gravity_alignment = transform::Rigid3d::Rotation(
        extrapolator_->EstimateGravityOrientation(time));

    // TODO(gaschler): This assumes that 'range_data_poses.back()' is at time
    // 'time'.
    // 以最后一个点的时间戳估计出的坐标为这帧数据的原点，range_data_poses是估计出来的，每次重新启动的origin都不固定
    accumulated_range_data_.origin = range_data_poses.back().translation();
    
    return AddAccumulatedRangeData(
        time,
        // 将点云变换到local原点处, 且姿态为0
        TransformToGravityAlignedFrameAndFilter(
          //重力对齐姿态       *         最后一个点的pose变换到local坐标系原点的坐标变换
            gravity_alignment.cast<float>() * range_data_poses.back().inverse(),// inverse() 返回平移和旋转
            accumulated_range_data_),
        gravity_alignment, sensor_duration);
  }

  return nullptr;
}

/**
 * @brief 进行扫描匹配, 将点云写入地图
 * 
 * @param[in] time 点云的时间戳
 * @param[in] gravity_aligned_range_data 原点位于local坐标系原点处的点云
 * @param[in] gravity_alignment 机器人当前姿态
 * @param[in] sensor_duration 2帧点云数据的时间差
 * @return std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult> 
 */
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddAccumulatedRangeData(
    const common::Time time,
    const sensor::RangeData& gravity_aligned_range_data,
    const transform::Rigid3d& gravity_alignment,
    const absl::optional<common::Duration>& sensor_duration) {
  // 如果处理完点云之后数据为空, 就报错. 使用单线雷达时不要设置min_z
  if (gravity_aligned_range_data.returns.empty()) {
    LOG(WARNING) << "Dropped empty horizontal range data.";
    return nullptr;
  }

  // Computes a gravity aligned pose prediction.
  // 进行位姿的预测, 先验位姿
  const transform::Rigid3d non_gravity_aligned_pose_prediction =
      extrapolator_->ExtrapolatePose(time); // 获取当前时刻位姿的预测
  // 将三维位姿先旋转到姿态为0, 再取xy坐标将三维位姿转成二维位姿
  const transform::Rigid2d pose_prediction = transform::Project2D(
      non_gravity_aligned_pose_prediction * gravity_alignment.inverse());

  // Step: 7 对 returns点云 进行自适应体素滤波，返回的点云的数据类型是PointCloud
  const sensor::PointCloud& filtered_gravity_aligned_point_cloud =
      sensor::AdaptiveVoxelFilter(gravity_aligned_range_data.returns,
                                  options_.adaptive_voxel_filter_options());
  if (filtered_gravity_aligned_point_cloud.empty()) {
    return nullptr;
  }

  // local map frame <- gravity-aligned frame
  // 扫描匹配, 进行点云与submap的匹配
  std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
      ScanMatch(time, pose_prediction, filtered_gravity_aligned_point_cloud);

  if (pose_estimate_2d == nullptr) {
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }

  // 将二维坐标旋转回之前的姿态
  const transform::Rigid3d pose_estimate =
      transform::Embed3D(*pose_estimate_2d) * gravity_alignment;
  // 校准位姿估计器
  extrapolator_->AddPose(time, pose_estimate);

  // Step: 8 将 原点位于local坐标系原点处的点云 变换成 原点位于匹配后的位姿处的点云
  sensor::RangeData range_data_in_local =
      TransformRangeData(gravity_aligned_range_data,
                         transform::Embed3D(pose_estimate_2d->cast<float>()));
  
  // 将校正后的雷达数据写入submap
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time, range_data_in_local, filtered_gravity_aligned_point_cloud,
      pose_estimate, gravity_alignment.rotation());

  // 计算耗时
  const auto wall_time = std::chrono::steady_clock::now();
  if (last_wall_time_.has_value()) {
    const auto wall_time_duration = wall_time - last_wall_time_.value();
    kLocalSlamLatencyMetric->Set(common::ToSeconds(wall_time_duration));
    if (sensor_duration.has_value()) {
      kLocalSlamRealTimeRatio->Set(common::ToSeconds(sensor_duration.value()) /
                                   common::ToSeconds(wall_time_duration));
    }
  }
  // 计算cpu耗时
  const double thread_cpu_time_seconds = common::GetThreadCpuTimeSeconds();
  if (last_thread_cpu_time_seconds_.has_value()) {
    const double thread_cpu_duration_seconds =
        thread_cpu_time_seconds - last_thread_cpu_time_seconds_.value();
    if (sensor_duration.has_value()) {
      kLocalSlamCpuRealTimeRatio->Set(
          common::ToSeconds(sensor_duration.value()) /
          thread_cpu_duration_seconds);
    }
  }
  last_wall_time_ = wall_time;
  last_thread_cpu_time_seconds_ = thread_cpu_time_seconds;

  // 返回结果 
  return absl::make_unique<MatchingResult>(
      MatchingResult{time, pose_estimate, std::move(range_data_in_local),
                     std::move(insertion_result)});
}

/**
 * @brief 将处理后的雷达数据写入submap
 * 
 * @param[in] time 点云的时间
 * @param[in] range_data_in_local 校正后的点云
 * @param[in] filtered_gravity_aligned_point_cloud 自适应体素滤波后的点云
 * @param[in] pose_estimate 扫描匹配后的三维位姿
 * @param[in] gravity_alignment 
 * @return std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult> 
 */
std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult>
LocalTrajectoryBuilder2D::InsertIntoSubmap(
    const common::Time time, const sensor::RangeData& range_data_in_local,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment) {
  // 如果移动距离过小, 或者时间过短, 不进行地图的更新
  if (motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }
  // 将点云数据写入到submap中
  std::vector<std::shared_ptr<const Submap2D>> insertion_submaps =
      active_submaps_.InsertRangeData(range_data_in_local);

  // 生成InsertionResult格式的数据进行返回
  return absl::make_unique<InsertionResult>(InsertionResult{
      std::make_shared<const TrajectoryNode::Data>(TrajectoryNode::Data{
          time,
          gravity_alignment,
          filtered_gravity_aligned_point_cloud,  // 这里存的是体素滤波后的点云, 不是校准后的点云
          {},  // 'high_resolution_point_cloud' is only used in 3D.
          {},  // 'low_resolution_point_cloud' is only used in 3D.
          {},  // 'rotational_scan_matcher_histogram' is only used in 3D.
          pose_estimate}),
      std::move(insertion_submaps)});
}

// 将IMU数据加入到Extrapolator中
void LocalTrajectoryBuilder2D::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";
  InitializeExtrapolator(imu_data.time);
  extrapolator_->AddImuData(imu_data);
}

// 将里程计数据加入到Extrapolator中
void LocalTrajectoryBuilder2D::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->AddOdometryData(odometry_data);
}

// 如果Extrapolator没有初始化就进行初始化
void LocalTrajectoryBuilder2D::InitializeExtrapolator(const common::Time time) {
  // 如果已经初始化过了就直接返回
  if (extrapolator_ != nullptr) {
    return;
  }

  // 注意 use_imu_based为true就会报错
  CHECK(!options_.pose_extrapolator_options().use_imu_based());
  // TODO(gaschler): Consider using InitializeWithImu as 3D does.

  // 初始化位姿推测器
  extrapolator_ = absl::make_unique<PoseExtrapolator>(
      ::cartographer::common::FromSeconds(options_.pose_extrapolator_options()
                                              .constant_velocity()
                                              .pose_queue_duration()), // 0.001s
      options_.pose_extrapolator_options()
          .constant_velocity()
          .imu_gravity_time_constant()); // 10
  // 添加初始位姿
  extrapolator_->AddPose(time, transform::Rigid3d::Identity());// 坐标000，旋转也是000的初始位姿
}

void LocalTrajectoryBuilder2D::RegisterMetrics(
    metrics::FamilyFactory* family_factory) {
  auto* latency = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_latency",
      "Duration from first incoming point cloud in accumulation to local slam "
      "result");
  kLocalSlamLatencyMetric = latency->Add({});
  auto* real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_real_time_ratio",
      "sensor duration / wall clock duration.");
  kLocalSlamRealTimeRatio = real_time_ratio->Add({});

  auto* cpu_real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_cpu_real_time_ratio",
      "sensor duration / cpu duration.");
  kLocalSlamCpuRealTimeRatio = cpu_real_time_ratio->Add({});
  auto score_boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_scores", "Local scan matcher scores",
      score_boundaries);
  kRealTimeCorrelativeScanMatcherScoreMetric =
      scores->Add({{"scan_matcher", "real_time_correlative"}});
  auto cost_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 100);
  auto* costs = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_costs", "Local scan matcher costs",
      cost_boundaries);
  kCeresScanMatcherCostMetric = costs->Add({{"scan_matcher", "ceres"}});
  auto distance_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 10);
  auto* residuals = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_residuals",
      "Local scan matcher residuals", distance_boundaries);
  kScanMatcherResidualDistanceMetric =
      residuals->Add({{"component", "distance"}});
  kScanMatcherResidualAngleMetric = residuals->Add({{"component", "angle"}});
}

}  // namespace mapping
}  // namespace cartographer
