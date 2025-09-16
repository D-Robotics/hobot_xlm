// Copyright (c) [2025] [Horizon Robotics][Horizon Bole].
//
// You can use this software according to the terms and conditions of
// the Apache v2.0.
// You may obtain a copy of Apache v2.0. at:
//
//     http: //www.apache.org/licenses/LICENSE-2.0
//
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
// NON-INFRINGEMENT, MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See Apache v2.0 for more details.

#ifndef LLM_ENGINE_XLM_INCLUDE_XLM_H_
#define LLM_ENGINE_XLM_INCLUDE_XLM_H_

#ifdef __cplusplus
extern "C" {  // C++ 环境下生效
#endif

#include <stdint.h>

typedef void *xlm_handle_t;

// sampling parameters
typedef struct common_params_sampling_s {
  int32_t top_k;  // <= 0 to use vocab size
  float top_p;    // 1.0 = disabled
  float min_p;    // 0.0 = disabled
  float temp;     // <= 0.0 to sample greedily, 0.0 to not output probabilities
  float typ_p;    // typical_p, 1.0 = disabled
  int32_t min_keep;
} common_params_sampling_t;

typedef enum xlm_model_type_e {
  XLM_MODEL_TYPE_LLAMA = 0,
  XLM_MODEL_TYPE_QWEN = 1,
  XLM_MODEL_TYPE_INTERNLM = 2,
  XLM_MODEL_TYPE_DEEPSEEK = 3,
  XLM_MODEL_TYPE_INTERNVL = 4
} xlm_model_type;

// Xlm common parameters
typedef struct xlm_common_params_s {
  const char *model_path;
  const char *token_config_path;
  const char *config_path;
  bool k_cache_int8;
  xlm_model_type model_type;
  int32_t context_size;
  common_params_sampling_t sampling;  // sampling parameters
  char *prompt_file;        // store the external prompt file name // NOLINT
  char *path_prompt_cache;  // path to file for saving/loading prompt eval state
                            // // NOLINT
} xlm_common_params_t;

typedef enum xlm_input_type_e {
  XLM_INPUT_PROMPT = 0,
  XLM_INPUT_TOKEN = 1,
  XLM_INPUT_IMAGE = 2
} xlm_input_type;

typedef struct xlm_input_token_s {
  int32_t *tokens;
  int32_t tokens_size;
} xlm_input_token_t;

typedef struct xlm_input_image_s {
  const char *prompt;
  const char *images_path;
  uint8_t *images;
  int32_t images_size;
} xlm_input_image_t;

typedef struct xlm_input_s {
  xlm_input_type type;
  bool new_chat;
  union {
    const char *prompt;
    xlm_input_token_t token;
    xlm_input_image_t image;
  };
  const char *system_prompt;
  const char *chat_template;
} xlm_input_t;

typedef struct xlm_result_s {
  char *text;
} xlm_result_t;

typedef enum xlm_state_e {
  XLM_STATE_START = 0,
  XLM_STATE_END = 1,
  XLM_STATE_RUNNING = 2,
  XLM_STATE_ERROR = 3
} xlm_state_t;

typedef void (*xlm_callback_t)(xlm_result_t *result, xlm_state_t state,
                               void *userdata);
// 创建默认参数数值
xlm_common_params_t xlm_create_default_param();
// 初始化实例
int xlm_init(xlm_common_params_t *param, xlm_callback_t callback,
             void **llm_handle);
// 启动推理包括一次完整的prefil和decode
int xlm_infer(xlm_handle_t handle, xlm_input_t *input, void *userdata);
// 异步处理，启动推理包括一次完整的prefil和decode
int xlm_infer_async(xlm_handle_t handle, xlm_input_t *input, void *userdata);
// 销毁实例
int xlm_destroy(xlm_handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif  // LLM_ENGINE_XLM_INCLUDE_XLM_H_
