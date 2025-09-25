// Copyright (c) 2025，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <stdio.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <fstream>

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/error/en.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ai_msgs/msg/perception_targets.hpp"
#include "ai_msgs/msg/perf.hpp"

#include "xlm.h"

#ifndef XLM_NODE_H_
#define XLM_NODE_H_

using rclcpp::NodeOptions;

using ai_msgs::msg::PerceptionTargets;

class XLMNode : public rclcpp::Node {
 public:
  XLMNode(const std::string &node_name,
                 const NodeOptions &options = NodeOptions());
  ~XLMNode() override;

  int LoadConfig(const std::string &json_file, xlm_common_params_t &params);

  int Init();

  int ChatFromLocal();

  int Chat(std::string& input_str, bool reset = true);

 private:

  void Run();

  static void Callback(xlm_result_s *result, xlm_state_e state, void *userdata);
 
  void OnCallback(xlm_result_s *result, xlm_state_e state);

  // string msg 控制话题信息
  rclcpp::Subscription<std_msgs::msg::String>::ConstSharedPtr
      ros_string_subscription_ = nullptr;
  std::string ros_string_sub_topic_name_ = "/prompt_text";
  void RosStringProcess(const std_msgs::msg::String::ConstSharedPtr msg);

  // 模型类别名
  std::string model_name_ = "DeepSeek_R1_Distill_Qwen_1.5B";
 
  std::string template_path_;

  // 用于预测的图片来源，0：本地交互；1：订阅string msg
  int feed_type_ = 0;

  // llm模型推理实例
  xlm_handle_t llm_handle_ = nullptr;
  
  std::queue<std::string> queue_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::atomic<bool> stop_thread_;
  
  std::thread thread_;

  // 发布AI消息的topic和发布者
  std::string ai_msg_pub_topic_name_ = "/generation/llm/deepseek";
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr ai_msg_publisher_ =
      nullptr;

  // 发布结果 message
  std::string text_msg_pub_topic_name_ = "/tts_text";
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr output_msg_publisher_ =
      nullptr;
};

#endif  // XLM_NODE_H_