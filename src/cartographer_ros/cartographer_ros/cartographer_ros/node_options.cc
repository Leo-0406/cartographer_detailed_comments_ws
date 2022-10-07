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
#include <vector>
#include "cartographer_ros/node_options.h"
#include "cartographer/common/configuration_file_resolver.h"    // 文件解析函数头文件
#include "cartographer/mapping/map_builder_interface.h"
#include "glog/logging.h"

namespace cartographer_ros {
/**
 * @brief 读取lua文件内容, 将lua文件的内容赋值给NodeOptions
 * 
 * @param lua_parameter_dictionary lua字典
 * @return NodeOptions 
 */
NodeOptions CreateNodeOptions(
    ::cartographer::common::LuaParameterDictionary* const
        lua_parameter_dictionary) {
          
    NodeOptions options;

    // 根据lua字典中的参数, 生成protobuf的序列化数据结构 proto::MapBuilderOptions
    options.map_builder_options =
        ::cartographer::mapping::CreateMapBuilderOptions(
            // lua_parameter_dictionary->GetDictionary获取map_builder的值，.get()在智能指针取值的时候用到
            lua_parameter_dictionary->GetDictionary("map_builder").get());
    // map_frame:地图坐标系的名字
    options.map_frame = lua_parameter_dictionary->GetString("map_frame");

    options.lookup_transform_timeout_sec =  // 查找tf时的超时时间  0.2
        lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");

    options.submap_publish_period_sec =     // 发布数据的时间间隔  0.3
        lua_parameter_dictionary->GetDouble("submap_publish_period_sec");

    options.pose_publish_period_sec =       // 发布pose的时间  5e-3
        lua_parameter_dictionary->GetDouble("pose_publish_period_sec");

    options.trajectory_publish_period_sec = // 发布轨迹的时间   30e-3
        lua_parameter_dictionary->GetDouble("trajectory_publish_period_sec");

    if (lua_parameter_dictionary->HasKey("publish_to_tf")) {
        options.publish_to_tf =         // 是否发布tf  true
            lua_parameter_dictionary->GetBool("publish_to_tf");
    }
    if (lua_parameter_dictionary->HasKey("publish_tracked_pose")) {
        options.publish_tracked_pose =   // 是否发布跟踪的位姿  false
            lua_parameter_dictionary->GetBool("publish_tracked_pose");
    }
    if (lua_parameter_dictionary->HasKey("use_pose_extrapolator")) {
        options.use_pose_extrapolator =  // 是否使用位姿推测器  true
            lua_parameter_dictionary->GetBool("use_pose_extrapolator");
    }
    return options;
}

/**
 * @brief 加载lua配置文件中的参数
 * 
 * @param[in] configuration_directory 配置文件所在目录
 * @param[in] configuration_basename 配置文件的名字
 * @return std::tuple<NodeOptions, TrajectoryOptions> 返回节点的配置与轨迹的配置
 */
std::tuple<NodeOptions, TrajectoryOptions> LoadOptions( 
    const std::string& configuration_directory,
    const std::string& configuration_basename) {
    // 获取配置文件所在的目录
    auto file_resolver =  // 生成的智能指针指向ConfigurationFileResolver这个类
        absl::make_unique<cartographer::common::ConfigurationFileResolver>(
            //  {configuration_directory}初始化string类型容器
            std::vector<std::string>{configuration_directory});     
        
    // 读取配置文件内容到code中
    const std::string code = // 使用类中的方法将配置文件的内容读取到code中
        file_resolver->GetFileContentOrDie(configuration_basename);

    // 根据给定的字符串, 生成一个lua字典
    cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
        code, std::move(file_resolver));

    // 创建元组tuple,元组定义了一个有固定数目元素的容器, 其中的每个元素类型都可以不相同
    // 将配置文件的内容填充进NodeOptions与TrajectoryOptions, 并返回
    return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary),
                            CreateTrajectoryOptions(&lua_parameter_dictionary));
} // std::tuple<NodeOptions, TrajectoryOptions> LoadOptions
}  // namespace cartographer_ros
