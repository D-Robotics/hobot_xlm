English| [简体中文](./README_cn.md)

Getting Started with Hobot Xlm Project
=======

# Feature Introduction

The hobot xlm package is an example project that integrates large language models (LLMs). 

# Development Environment

- Programming Language: C/C++
- Development Platform: S100/S100P
- System Version: Ubuntu 22.04
- Compilation Toolchain: Linaro GCC 11.4.0

# Compilation Options

1. Compilation Environment Verification

- The RDK S100 Ubuntu system is installed on the board.
- The current compilation terminal has set up the TogetherROS environment variable: `source PATH/setup.bash`. Where PATH is the installation path of TogetherROS.
- The ROS2 compilation tool colcon is installed. If the installed ROS does not include the compilation tool colcon, it needs to be installed manually. Installation command for colcon: `pip install -U colcon-common-extensions`.
- The dnn node package has been compiled.

2. Compilation

- Compilation command: `colcon build --packages-select hobot_xlm`

## Docker Cross-Compilation for RDK S100 Version

1. Compilation Environment Verification

- Compilation within docker, and TogetherROS has been installed in the docker environment. For instructions on docker installation, cross-compilation, TogetherROS compilation, and deployment, please refer to the README.md in the robot development platform's robot_dev_config repo.
- The dnn node package has been compiled.
- The hbm_img_msgs package has been compiled (see Dependency section for compilation methods).

2. Compilation

- Compilation command:

  ```shell
  # RDK S100
  bash robot_dev_config/build.sh -p S100 -s hobot_xlm
  ```

# Notes

## Parameters

| Parameter Name      | Explanation                            | Mandatory            | Default Value       | Remarks                                                                 |
| ------------------- | -------------------------------------- | -------------------- | ------------------- | ----------------------------------------------------------------------- |
| feed_type           | Data source, 0: vlm local; 1: vlm subscribe; 2: llm subscribe  | No                   | 0                   |                                                                         |
| model_name | language model name | No | DeepSeek_R1_Distill_Qwen_1.5B | Only Support "DeepSeek_R1_Distill_Qwen_1.5B", "DeepSeek_R1_Distill_Qwen_7B" |
| ai_msg_pub_topic_name | Topic name for publishing intelligent results | No                   | /generation/llm/deepseek | |
| text_msg_pub_topic_name | Topic name for publishing intelligent results for tts | No                   | /tts_text | |
| ros_string_sub_topic_name | Topic name for subscribing string msg to set user prompt| No                   | /prompt_text | |

## Instructions

- Prompts Publishing: hobot_llamacpp relies on user prompt from ros2 string msg messages. There is an example of how to use the string msg topic, where /prompt_text is the topic name. The data field contains a string that sets the prompt for the language model.

```shell
ros2 topic pub --once /prompt_text std_msgs/msg/String "{data: 'hello'}"
```

- Intermediate results Subscription: The hobot_llamacpp inference outputs text results. The model does not directly output the complete result; instead, the intermediate inference results can be promptly sent to the speech module for output.

```shell
ros2 topic echo /tts_text
```

- Final results Subscription:

```shell
ros2 topic echo /generation/llm/deepseek
```

# Running

## Prepare the models

- DeepSeek_R1_Distill_Qwen_1.5B

```shell
wget -c ftp://oeftp@sdk.d-robotics.cc/oe_llm/model/DeepSeek_R1_Distill_Qwen_1.5B_1024.hbm --ftp-password=Oeftp~123$%
```

- DeepSeek_R1_Distill_Qwen_7B

```shell
wget -c ftp://oeftp@sdk.d-robotics.cc/oe_llm/model/DeepSeek_R1_Distill_Qwen_7B_1024.hbm --ftp-password=Oeftp~123$%
```

## Running on RDK S100 Ubuntu System

Running method 1, use the executable file to start:
```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source ./install/local_setup.bash
lib=./install/lib/hobot_xlm/lib
export LD_LIBRARY_PATH=${lib}:${LD_LIBRARY_PATH}
# The config includes models used by the example and local images for filling
cp -r install/lib/hobot_xlm/config/ .

# Run mode 1:Use local chat
ros2 run hobot_xlm hobot_xlm --ros-args -p feed_type:=0 -p model_name:="DeepSeek_R1_Distill_Qwen_1.5B"

# Run mode 2: sub prompt
ros2 run hobot_xlm hobot_xlm --ros-args -p feed_type:=1 -p model_name:="DeepSeek_R1_Distill_Qwen_1.5B"

ros2 topic pub --once /prompt_text std_msgs/msg/String "{data: ""1258+1485x3等于多少？""}"
```

Running method 2, use a launch file:

```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source ./install/setup.bash
# Copy the configuration based on the actual installation path
cp -r install/lib/hobot_xlm/config/ .

# Start the launch file, run reid node only.
ros2 launch hobot_xlm llm_chat.launch.py xlm_model_name:="DeepSeek_R1_Distill_Qwen_1.5B"
```

# Results Analysis

## RDK S100 Result

Run：`ros2 launch hobot_xlm llm_chat.launch.py xlm_model_name:="DeepSeek_R1_Distill_Qwen_1.5B""`


```shell
[UCP]: log level = 3
[UCP]: UCP version = 3.6.1
[VP]: log level = 3
[DNN]: log level = 3
[HPL]: log level = 3
[UCPT]: log level = 6
[WARN] [1757949703.788157149] [xlm_node]: This is hobot xlm node!
[WARN] [1757949703.800199173] [xlm_node]: Parameter:
 feed_type(0:local, 1:sub): 0
 model_name: DeepSeek_R1_Distill_Qwen_1.5B
 ai_msg_pub_topic_name: /generation/lanaguage/deepseek
 text_msg_pub_topic_name: /tts_text
 ros_string_sub_topic_name: /prompt_text
[WARN] [1757949703.800428372] [xlm_node]: Model Parameter:
 model_path:   ./DeepSeek_R1_Distill_Qwen_1.5B_1024.hbm
 token_path:   ./config/DeepSeek_R1_Distill_Qwen_1.5B_config/
 k_cache_int8: 0
 model_type:   3
 context_size: 1024
 prompt_file:
 path_prompt_cache:
 sampling: {
     top_k:    3
     top_p:    0.95
     min_p:    0.1
     temp:     0.1
     typ_p:    1
     min_keep: 5
 }
[BPU][[BPU_MONITOR]][281473285378048][INFO]BPULib verison(2, 1, 2)[0d3f195]!
[DNN] HBTL_EXT_DNN log level:6
[DNN]: 3.6.1_(4.2.7post0.dev202307211111+6aaae37 HBRT)
[WARN] [1757949705.795194210] [xlm_node]: model init successed!
板端大模型多轮对话交互demo，请输入你的问题并按下回车
- 退出请输入exit
- 清除缓存请输入reset
[User] <<< 1258+1485x3等于多少？
[Assistant] >>> ...

**Step 1: Identify the Components**

First, identify the numbers and operations in the expression:
1258 + 1485 × 3

**Step 2: Perform the Multiplication**

Next, perform the multiplication part of the expression:
1485 × 3 = 4455

**Step 3: Add the Result to 1258**

Finally, add the result of the multiplication to 1258:
1258 + 4455 = 5713

**Final Answer:**
The result of 1258 + 1485 × 3 is 5713.
</think>

**Solution:**

We need to evaluate the expression:
\[ 1258 + 1485 \times 3 \]

**Step 1: Perform the Multiplication**

First, calculate the multiplication part of the expression:
\[ 1485 \times 3 = 4455 \]

**Step 2: Add the Result to 1258**

Next, add the result of the multiplication to 1258:
\[ 1258 + 4455 = 5713 \]

**Final Answer:**
\[
\boxed{5713}
\]
Performance prefill: 941.18tokens/s    decode: 22.63tokens/s
```