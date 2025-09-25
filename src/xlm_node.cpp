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
#include <string>
#include <unistd.h>
#include <vector>
#include <random>

#include "rclcpp/rclcpp.hpp"

#include "include/util.h"
#include "include/xlm_node.h"

static char* read_chat_template_file(const char *filepath) {
  FILE *file = fopen(filepath, "r");
  if (!file) {
      return NULL;
  }

  fseek(file, 0, SEEK_END);
  long filesize = ftell(file);
  rewind(file);
  char *content = (char *)malloc(filesize + 1);
  if (!content) {
      fclose(file);
      return NULL;
  }

  size_t read_size = fread(content, 1, filesize, file);
  content[read_size] = '\0';

  fclose(file);
  return content;
}

void callback(xlm_result_s *result, xlm_state_e state, void *userdata) {
  if (state == XLM_STATE_END) {
    std::cout << "\n[User] <<< " << std::flush;
  } else if (state == XLM_STATE_ERROR) {
    std::cout << "run error" << std::endl;
  } else if (state == XLM_STATE_START) {
    std::cout << "[Assistant] >>> " << result->text << std::flush;
  } else {
    std::cout << result->text << std::flush;
  }
}

XLMNode::XLMNode(const std::string &node_name,
                               const NodeOptions &options)
    : rclcpp::Node(node_name, options) {
  // 更新配置
  this->declare_parameter<int>("feed_type", feed_type_);
  this->declare_parameter<std::string>("model_name", model_name_);
  this->declare_parameter<std::string>("ai_msg_pub_topic_name",
                                       ai_msg_pub_topic_name_);
  this->declare_parameter<std::string>("text_msg_pub_topic_name",
                                      text_msg_pub_topic_name_);
  this->declare_parameter<std::string>("ros_string_sub_topic_name",
                                       ros_string_sub_topic_name_);

  this->get_parameter<int>("feed_type", feed_type_);
  this->get_parameter<std::string>("model_name", model_name_);
  this->get_parameter<std::string>("ai_msg_pub_topic_name", ai_msg_pub_topic_name_);
  this->get_parameter<std::string>("text_msg_pub_topic_name", text_msg_pub_topic_name_);
  this->get_parameter<std::string>("ros_string_sub_topic_name", ros_string_sub_topic_name_);

  {
    std::stringstream ss;
    ss << "Parameter:"
       << "\n feed_type(0:local, 1:sub): " << feed_type_
       << "\n model_name: " << model_name_
       << "\n ai_msg_pub_topic_name: " << ai_msg_pub_topic_name_
       << "\n text_msg_pub_topic_name: " << text_msg_pub_topic_name_
       << "\n ros_string_sub_topic_name: " << ros_string_sub_topic_name_;
    RCLCPP_WARN(rclcpp::get_logger("xlm_node"), "%s", ss.str().c_str());
  }

  if (Init() != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("xlm_node"), "model init failed!");
    return;
  }

  if (feed_type_ == 0) {
    ChatFromLocal();
  } else if (feed_type_ == 1) {
    thread_ = std::thread(&XLMNode::Run, this);
    // 创建AI消息的发布者
    ai_msg_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
        ai_msg_pub_topic_name_, 10);

    // 创建string消息的发布者
    output_msg_publisher_ = this->create_publisher<std_msgs::msg::String>(
      text_msg_pub_topic_name_, 10);

    // 创建string消息的订阅者
    ros_string_subscription_ = this->create_subscription<std_msgs::msg::String>(
            ros_string_sub_topic_name_, 10,
            std::bind(
                &XLMNode::RosStringProcess, this, std::placeholders::_1));
  }
}

XLMNode::~XLMNode() {
  stop_thread_ = true;
  cv_.notify_all();
  xlm_destroy(&llm_handle_);
  if (thread_.joinable()) thread_.join();
}

int XLMNode::LoadConfig(const std::string &json_file, xlm_common_params_t &params) {
    FILE *fp = fopen(json_file.c_str(), "r");
    if (!fp) {          
      RCLCPP_ERROR(rclcpp::get_logger("xlm_node"),
          "Cannot open JSON file: %s.", json_file.c_str());
      return -1;
    }

    char readBuffer[65536];
    rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    rapidjson::Document doc;
    doc.ParseStream(is);
    fclose(fp);

    if (doc.HasParseError()) {
      std::stringstream ss;
      ss << "JSON parse error: "
          << rapidjson::GetParseError_En(doc.GetParseError())
          << " at offset " << doc.GetErrorOffset();
      RCLCPP_ERROR(rclcpp::get_logger("xlm_node"), "%s", ss.str().c_str());
      return -1;
    }

    if (doc.HasMember("model_path")) params.model_path = strdup(doc["model_path"].GetString());
    if (doc.HasMember("token_config_path")) params.token_config_path = strdup(doc["token_config_path"].GetString());
    if (doc.HasMember("config_path")) params.config_path = strdup(doc["config_path"].GetString());
    if (doc.HasMember("k_cache_int8")) params.k_cache_int8 = doc["k_cache_int8"].GetBool();
    if (doc.HasMember("model_type")) params.model_type = static_cast<xlm_model_type>(doc["model_type"].GetInt());
    if (doc.HasMember("context_size")) params.context_size = doc["context_size"].GetInt();

    if (doc.HasMember("sampling")) {
        const auto &s = doc["sampling"];
        if (s.HasMember("top_k")) params.sampling.top_k = s["top_k"].GetInt();
        if (s.HasMember("top_p")) params.sampling.top_p = s["top_p"].GetFloat();
        if (s.HasMember("min_p")) params.sampling.min_p = s["min_p"].GetFloat();
        if (s.HasMember("temp")) params.sampling.temp = s["temp"].GetFloat();
        if (s.HasMember("typ_p")) params.sampling.typ_p = s["typ_p"].GetFloat();
        if (s.HasMember("min_keep")) params.sampling.min_keep = s["min_keep"].GetInt();
        if (s.HasMember("penalty_last_n")) params.sampling.penalty_last_n = s["penalty_last_n"].GetInt();
        if (s.HasMember("penalty_freq")) params.sampling.penalty_freq = s["penalty_freq"].GetFloat();
        if (s.HasMember("penalty_present")) params.sampling.penalty_present = s["penalty_present"].GetFloat();
        if (s.HasMember("penalty_repeat")) params.sampling.penalty_repeat = s["penalty_repeat"].GetFloat();
    }

    if (doc.HasMember("prompt_file")) params.prompt_file = strdup(doc["prompt_file"].GetString());
    if (doc.HasMember("path_prompt_cache")) params.path_prompt_cache = strdup(doc["path_prompt_cache"].GetString());

    return 0;
}

int XLMNode::Init() {

  // 初始化参数
  xlm_common_params_t params = xlm_create_default_param();

  std::string config_file = "./config/" + model_name_ + "_config/param.json";
  template_path_ = "./config/" + model_name_ + "_config/" + model_name_ + ".jinja";

  LoadConfig(config_file, params);

  {
    std::stringstream ss;
    ss << "Model Parameter:"
       << "\n model_path:   " << params.model_path
       << "\n token_path:   " << params.token_config_path
       << "\n k_cache_int8: " << params.k_cache_int8
       << "\n model_type:   " << params.model_type
       << "\n context_size: " << params.context_size
       << "\n prompt_file:  " << (params.prompt_file ? params.prompt_file : "null")
       << "\n path_prompt_cache: " << (params.path_prompt_cache ? params.path_prompt_cache : "null")
       << "\n sampling: {"
       << "\n     top_k:    " << params.sampling.top_k
       << "\n     top_p:    " << params.sampling.top_p
       << "\n     min_p:    " << params.sampling.min_p
       << "\n     temp:     " << params.sampling.temp
       << "\n     typ_p:    " << params.sampling.typ_p
       << "\n     min_keep: " << params.sampling.min_keep
       << "\n     penalty_last_n: " << params.sampling.penalty_last_n
       << "\n     penalty_freq:   " << params.sampling.penalty_freq
       << "\n     penalty_present:" << params.sampling.penalty_present
       << "\n     penalty_repeat: " << params.sampling.penalty_repeat
       << "\n }";
    RCLCPP_WARN(rclcpp::get_logger("xlm_node"), "%s", ss.str().c_str());
  }

  int ret = 0;
  if (feed_type_ == 0) {
    ret = xlm_init(&params, callback, &llm_handle_);
  } else {
    ret = xlm_init(&params, &XLMNode::Callback, &llm_handle_);
  }
  return ret;
}

void XLMNode::Run() {
  std::string sub_string;
  std::string result;
  while (!stop_thread_) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this]() { return !queue_.empty() || stop_thread_; });

    while (!queue_.empty()) {
      std::string text = queue_.front();
      queue_.pop();
      lock.unlock();

      if (text == "END") {
        std::stringstream ss;
        ss << result;
        RCLCPP_WARN(rclcpp::get_logger("xlm_node"), "\n%s", ss.str().c_str());

        ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(
          new ai_msgs::msg::PerceptionTargets());
        ai_msgs::msg::Target target;
        target.set__type(result);
        pub_data->targets.emplace_back(std::move(target));
        ai_msg_publisher_->publish(std::move(pub_data));

      } else if (text == "START") {
        sub_string = "";
        result = "";
      } else {
        bool hasChinese = false;
        bool hasPunctuation = false;
        std::string filtered = filterChineseEnglishAndPunctuation(text, hasChinese, hasPunctuation);

        sub_string += filtered;
        result += text;

        // std::cout << "\n" << text << " " << hasChinese << " " << hasPunctuation << " " << filtered << std::endl;
        // std::cout << "sub_string: " << sub_string << std::endl;
        if (hasPunctuation) {
          if (sub_string == "") continue;
          std_msgs::msg::String::UniquePtr msg(new std_msgs::msg::String());
          msg->data = sub_string;
          output_msg_publisher_->publish(std::move(msg));
          sub_string = "";
        }
      }
      lock.lock();
    }
  }
}

void XLMNode::RosStringProcess(
    const std_msgs::msg::String::ConstSharedPtr msg) {
  if (!msg) {
    RCLCPP_DEBUG(rclcpp::get_logger("xlm_node"), "Get string failed");
    return;
  }

  if (!rclcpp::ok()) {
    return;
  }

  std::stringstream ss;
  ss << "Recved string data: " << msg->data;
  RCLCPP_WARN(rclcpp::get_logger("xlm_node"), "%s", ss.str().c_str());

  std::string input_str = msg->data;
  Chat(input_str, true);
}

void XLMNode::Callback(xlm_result_s *result, xlm_state_e state, void *userdata) {
  // 先把 userdata 转回类对象指针
  XLMNode *self = static_cast<XLMNode*>(userdata);
  // 调用实例方法来处理结果
  self->OnCallback(result, state);
}

// 实例方法（你原来的逻辑移到这里）
void XLMNode::OnCallback(xlm_result_s *result, xlm_state_e state) {
  if (state == XLM_STATE_END) {
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.push("END");
    cv_.notify_one();
  } else if (state == XLM_STATE_ERROR) {
    // std::cout << "run error" << std::endl;
  } else if (state == XLM_STATE_START) {
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.push("START");
    cv_.notify_one();
  } else {
    std::cout << result->text << std::flush;
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.push(result->text);
    cv_.notify_one();
  }
}

int XLMNode::ChatFromLocal() {
  
  std::string input_str;
  std::cout << "板端大模型多轮对话交互demo，请输入你的问题并按下回车 \n"
            << "- 退出请 Ctrl C\n"
            << "- 清除缓存请输入 reset" << std::endl;

  std::cout << "[User] <<< " << std::flush;

  bool reset = true;
  while (std::getline(std::cin, input_str)) {
    if (input_str == "reset") {
      std::cout << "[User] <<< " << std::flush;
      continue;
    } 

    Chat(input_str, reset);
    reset = false;
  }

  return 0;
}

int XLMNode::Chat(std::string& input_str, bool reset) {

  const char *chat_template_path = template_path_.c_str();

  xlm_input_t input;
  memset(&input, 0, sizeof(xlm_input_t));

  input.request_num = 1;
  xlm_lm_request_t *requests =
      (xlm_lm_request_t *)malloc(sizeof(xlm_lm_request_t) * input.request_num);
  input.requests = requests;

  xlm_lm_request_t *request = &input.requests[0];
  memset(request, 0, sizeof(xlm_lm_request_t));

  request->type = XLM_INPUT_PROMPT;
  request->new_chat = reset;
  request->system_prompt = NULL;

  if (chat_template_path && strlen(chat_template_path) > 0) {
    request->chat_template = read_chat_template_file(chat_template_path);
  }

  request->infer_backend = XLM_INFER_BACKEND_BPU_ANY;

  request->prompt = input_str.c_str();
  xlm_infer_async(llm_handle_, &input, this);

  if (request->chat_template) {
    free((void *)request->chat_template);
    request->chat_template = NULL;
  }
  free(requests);

  return 0;
}