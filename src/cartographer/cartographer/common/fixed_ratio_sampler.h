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

#ifndef CARTOGRAPHER_COMMON_FIXED_RATIO_SAMPLER_H_
#define CARTOGRAPHER_COMMON_FIXED_RATIO_SAMPLER_H_

#include <string>

#include "cartographer/common/port.h"

namespace cartographer {
namespace common {

// Signals when a sample should be taken from a stream of data to select a
// uniformly distributed fraction of the data.
// 当应该从数据流中抽取样本以选择数据的均匀分布部分时发出信号
class FixedRatioSampler {
public:
  explicit FixedRatioSampler(double ratio);
  ~FixedRatioSampler();
  // 禁止生成默认的赋值构造函数
  FixedRatioSampler(const FixedRatioSampler&) = delete;
  FixedRatioSampler& operator=(const FixedRatioSampler&) = delete;

  // Returns true if this pulse should result in an sample.
  // 在比例小于ratio_时返回true, ratio_设置为1时都返回true, 也就是说使用所有的数据
  bool Pulse(); // 暂停函数

  // Returns a debug string describing the current ratio of samples to pulses.
  std::string DebugString();

 private:
  // Sampling occurs if the proportion of samples to pulses drops below this
  // number.
  const double ratio_;

  int64 num_pulses_ = 0;
  int64 num_samples_ = 0;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_FIXED_RATIO_SAMPLER_H_
