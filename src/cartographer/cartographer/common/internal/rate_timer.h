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

#ifndef CARTOGRAPHER_COMMON_RATE_TIMER_H_
#define CARTOGRAPHER_COMMON_RATE_TIMER_H_

#include <chrono>
#include <deque>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"

namespace cartographer {
namespace common {

// Computes the rate at which pulses come in.
template <typename ClockType = std::chrono::steady_clock>
class RateTimer {
 public:
  // Computes the rate at which pulses come in over 'window_duration' in wall
  // time.
  explicit RateTimer(const common::Duration window_duration)
      : window_duration_(window_duration) {}
  ~RateTimer() {}

  RateTimer(const RateTimer&) = delete;
  RateTimer& operator=(const RateTimer&) = delete;

  // Returns the pulse rate in Hz.
  // 计算平均频率,数据的个数 - 1 除以 开始与结束的数据间隔
  double ComputeRate() const {
    if (events_.empty()) {
      return 0.;
    }
    return static_cast<double>(events_.size() - 1) /
           common::ToSeconds((events_.back().time - events_.front().time));
  }

  // Returns the ratio of the pulse rate (with supplied times) to the wall time
  // rate. For example, if a sensor produces pulses at 10 Hz, but we call Pulse
  // at 20 Hz wall time, this will return 2.
  // 返回脉搏率（提供时间）与wall time率的比率
  // 例如, 如果传感器产生 10 Hz 的脉冲, 但我们将 Pulse 称为 20 Hz 墙时间, 这将返回 2
  double ComputeWallTimeRateRatio() const {
    if (events_.empty()) {
      return 0.;
    }                                // time：传感器的时间
    return common::ToSeconds((events_.back().time - events_.front().time)) /
           common::ToSeconds(events_.back().wall_time -
                             events_.front().wall_time);
  }

  // Records an event that will contribute to the computed rate.
  // 对数据队列进行更新
  void Pulse(common::Time time) {
    // 将传入的时间放入队列中
    events_.push_back(Event{time, ClockType::now()});
    // 删除队列头部数据,直到队列中最后与最前间的时间间隔小于window_duration_
    while (events_.size() > 2 &&              //  C++系统的时间
           (events_.back().wall_time - events_.front().wall_time) >
                //    15秒
               window_duration_) {
      events_.pop_front();
    }
  }

  // Returns a debug string representation.
  std::string DebugString() const {
    if (events_.size() < 2) {
      return "unknown";
    }

    // c++11: std::fixed 与 std::setprecision(2) 一起使用, 表示输出2位小数点的数据

    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << ComputeRate() << " Hz "
        << DeltasDebugString() << " (pulsed at "
        << ComputeWallTimeRateRatio() * 100. << "% real time)";
    return out.str();
  }

 private:
  struct Event {
    common::Time time;
    typename ClockType::time_point wall_time;
  };

  // Computes all differences in seconds between consecutive pulses.
  // 返回每2个数据间的时间间隔
  std::vector<double> ComputeDeltasInSeconds() const {
    CHECK_GT(events_.size(), 1);
    const size_t count = events_.size() - 1;
    std::vector<double> result;
    result.reserve(count);
    for (size_t i = 0; i != count; ++i) {
      result.push_back(
          common::ToSeconds(events_[i + 1].time - events_[i].time));
    }
    return result;
  }

  // Returns the average and standard deviation of the deltas.
  // 计算数据时间间隔的均值与标准差
  std::string DeltasDebugString() const {
    const auto deltas = ComputeDeltasInSeconds();
    const double sum = std::accumulate(deltas.begin(), deltas.end(), 0.);
    // 计算均值
    const double mean = sum / deltas.size();

    double squared_sum = 0.;
    for (const double x : deltas) {
      squared_sum += common::Pow2(x - mean);
    }
    // 计算标准差
    const double sigma = std::sqrt(squared_sum / (deltas.size() - 1));

    std::ostringstream out;
    out << std::scientific << std::setprecision(2) << mean << " s +/- " << sigma
        << " s";
    return out.str();
  }

  std::deque<Event> events_;
  const common::Duration window_duration_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_RATE_TIMER_H_
