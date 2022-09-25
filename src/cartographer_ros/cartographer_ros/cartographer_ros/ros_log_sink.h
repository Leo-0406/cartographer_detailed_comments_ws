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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_LOG_SINK_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_LOG_SINK_H

#include <ctime>
#include "glog/logging.h"

namespace cartographer_ros {

// Makes Google logging use ROS logging for output while an instance of this
// class exists.
/**
 * @brief 自定义的输出日志的方式: 使用ROS_INFO进行glog消息的输出
 */
// 继承google::LogSink类
  class ScopedRosLogSink : public 
   {
    public:
      ScopedRosLogSink();
      ~ScopedRosLogSink() override;
      // 函数实现
      void send(::google::LogSeverity severity,   // 消息级别
                const char* filename,             // 全路径文件名
                const char* base_filename,        // 文件名
                int line,                         // 消息所在的文件行数
                const struct std::tm* tm_time,    // 消息的时间
                const char* message,              // 消息数据本体
                size_t message_len) override;     // 消息长度

      void WaitTillSent() override;

    private:
      bool will_die_;
  }; // class ScopedRosLogSink
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_LOG_SINK_H
