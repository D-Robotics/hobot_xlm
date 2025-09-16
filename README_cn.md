[English](./README.md) | 简体中文

# 功能介绍

语言大模型算法示例, 是基于地瓜机器人面向大算力平台提供的大模型工具链的量化部署方案, 通过对文本的输入、推理、输出大模型推理结果。

# 开发环境

- 编程语言: C/C++
- 开发平台: S100/S100P
- 系统版本：Ubuntu 22.04
- 编译工具链: Linux GCC 11.4.0

# 编译

- S100版本：支持在 S100 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

同时支持通过编译选项控制编译pkg的依赖和pkg的功能。

## 依赖库

- opencv:3.4.5

ros package：

- ai_msgs

## 编译选项

1、SHARED_MEM

- shared mem（共享内存传输）使能开关, 默认打开（ON）, 编译时使用-DSHARED_MEM=OFF命令关闭。
- 如果打开, 编译和运行会依赖hbm_img_msgs pkg, 并且需要使用tros进行编译。
- 如果关闭, 编译和运行不依赖hbm_img_msgs pkg, 支持使用原生ros和tros进行编译。
- 对于shared mem通信方式, 当前只支持订阅nv12格式图片。

## RDK Ubuntu系统上编译

1、编译环境确认

- 板端已安装S100 Ubuntu系统。
- 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
- 已安装ROS2编译工具colcon。安装的ROS不包含编译工具colcon, 需要手动安装colcon。colcon安装命令：`pip install -U colcon-common-extensions`
- 已编译dnn node package

2、编译

- 编译命令：`colcon build --packages-select reid`

## docker交叉编译 S100版本

1、编译环境确认

- 在docker中编译, 并且docker中已经安装好TogetherROS。docker安装、交叉编译说明、TogetherROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。

2、编译

- 编译命令：

  ```shell
  # RDK S100
  bash robot_dev_config/build.sh -p S100 -s xlm_node
  ```

- 编译选项中默认打开了shared mem通信方式。

## 注意事项

# 使用介绍

## 依赖

- mipi_cam package：发布图片msg
- usb_cam package：发布图片msg
- websocket package：渲染图片和ai感知msg

## 参数

| 参数名             | 解释                                  | 是否必须             | 数值类型 | 默认值                 |
| ------------------ | ------------------------------------- | -------------------- | ------------------- | ----------------------------------------------------------------------- |
| feed_type           | int         | 本地/订阅推理模式。0：本地对话；1：订阅话题                                                                         | 否       | 0/1                  | 0                            |
| model_name        | std::string | 模型名  | 否 | 目前支持。"DeepSeek_R1_Distill_Qwen_1.5B", "DeepSeek_R1_Distill_Qwen_7B" | "DeepSeek_R1_Distill_Qwen_1.5B" |
| ai_msg_pub_topic_name | 发布智能结果的topicname | 否                   | //generation/llm/deepseek | |
| text_msg_pub_topic_name | 发布智能结果的topicname,中间结果 | 否                   | /tts_text | |
| ros_img_sub_topic_name | 接收ros图片话题名 | 否                   | /image | |
| ros_string_sub_topic_name | 接收string消息话题获得文本提示词 | 否                   | /prompt_text | |


## 模型下载

- DeepSeek_R1_Distill_Qwen_1.5B

```shell
wget -c ftp://oeftp@sdk.d-robotics.cc/oe_llm_v0.9.0/DeepSeek_R1_Distill_Qwen_1.5B_4096.hbm --ftp-password=Oeftp~123$%
```

- DeepSeek_R1_Distill_Qwen_7B

```shell
wget -c ftp://oeftp@sdk.d-robotics.cc/oe_llm_v0.9.0/DeepSeek_R1_Distill_Qwen_7B_1024.hbm --ftp-password=Oeftp~123$%
```

## 运行

- 下载 hobot_xlm 使用到的模型在当前路径下。

- 编译成功后, 将生成的install路径拷贝到地平线RDK上（如果是在RDK上编译, 忽略拷贝步骤）, 并执行如下命令运行。

## RDK S100 Ubuntu系统上运行

运行方式1, 使用可执行文件启动：
```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source ./install/setup.bash
lib=./install/lib/hobot_xlm/lib
export LD_LIBRARY_PATH=${lib}:${LD_LIBRARY_PATH}
# config中为示例使用的模型配置文件
cp -r install/lib/hobot_xlm/config/ .

# 运行模式1：本地对话
ros2 run hobot_xlm hobot_xlm --ros-args -p feed_type:=0 -p model_name:="DeepSeek_R1_Distill_Qwen_1.5B"

# 运行模式2：订阅提示词
ros2 run hobot_xlm hobot_xlm --ros-args -p feed_type:=1 -p ros_string_sub_topic_name:="/prompt_text" -p model_name:="DeepSeek_R1_Distill_Qwen_1.5B"

# 同时使用
ros2 topic pub --once /prompt_text std_msgs/msg/String "{data: ""1258+1485x3等于多少？""}"
```

运行方式2, 使用launch文件启动：
```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
source ./install/setup.bash
# config中为示例使用的模型, 根据实际安装路径进行拷贝
# 如果是板端编译（无--merge-install编译选项）, 拷贝命令为cp -r install/PKG_NAME/lib/PKG_NAME/config/ ., 其中PKG_NAME为具体的package名。
cp -r install/lib/hobot_xlm/config/ .

# 运行模式1：本地对话
ros2 launch hobot_xlm llm_chat.launch.py
```

# 结果分析

## S100 结果展示

log：

运行命令：`ros2 launch hobot_xlm llm_chat.launch.py xlm_model_name:="DeepSeek_R1_Distill_Qwen_1.5B""`

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
 model_path:   ./DeepSeek_R1_Distill_Qwen_1.5B_4096.hbm
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