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

// 判断一个字符是否是英文（A-Z, a-z）
bool isEnglish(const std::string& str, size_t i) {
    if (i >= str.size()) return false;

    unsigned char c = str[i];

    // 英文字母：A-Z (0x41-0x5A), a-z (0x61-0x7A)
    if ((c >= 0x41 && c <= 0x5A) || (c >= 0x61 && c <= 0x7A) || c == ' ') {
        return true;
    }
    return false;
}

// 判断一个字符是否是数字（0-9）
bool isDigitChar(const std::string& str, size_t i) {
    if (i >= str.size()) return false;

    unsigned char c = str[i];
    return (c >= '0' && c <= '9') || c == '+' || c == '-' || c == '/' || c == '*';
}

// 判断一个字符是否是中文（UTF-8 三字节范围）
bool isChinese(const std::string& str, size_t i) {
  if (i + 2 >= str.size()) return false;

  unsigned char c1 = str[i];
  unsigned char c2 = str[i + 1];
  unsigned char c3 = str[i + 2];

  // 中文 UTF-8 范围是 E4 B8 80 到 E9 BE A5
  return (c1 >= 0xE4 && c1 <= 0xE9);
}

// 判断是否是中文逗号或句号（UTF-8编码：，=E3 80 81，。=E3 80 82）
bool isChinesePunctuation(const std::string& str, size_t i) {
  if (i + 2 >= str.size()) return false;

  return (str[i] == char(0xE3) && str[i + 1] == char(0x80) &&
          (str[i + 2] == char(0x81) || str[i + 2] == char(0x82)));
}

// 判断是否是英文逗号、句号、感叹号
bool isEnglishPunctuation(const std::string& str, size_t i) {
    if (i >= str.size()) return false;

    unsigned char c = str[i];
    return (c == ',' || c == '.' || c == '!');
}

std::string filterChineseEnglishAndPunctuation(const std::string& input, bool& hasChinese, bool& hasPunctuation) {
  std::string result;
  hasChinese = false;
  hasPunctuation = false;

  for (size_t i = 0; i < input.size(); ) {
      if (isChinese(input, i)) {
          hasChinese = true;
          result += input.substr(i, 3);
          i += 3;
      } else if (isEnglish(input, i)) {
          result += input.substr(i, 1);
          i += 1;
      } else if (isDigitChar(input, i)) {
          result += input.substr(i, 1);
          i += 1;
      }
      else {
        hasPunctuation = true;
        if (isChinesePunctuation(input, i)) {
          result += input.substr(i, 3);
          i += 3;
        } else if (isEnglishPunctuation(input, i)) {
          result += input.substr(i, 1);
          i += 1;
        } else {
          // 英文字符/符号等：跳过
          unsigned char c = input[i];
          if (c < 0x80) {
              if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')) {
                  // 保留英文字母
                  result += c;
              }
              ++i;
          } else if ((c & 0xE0) == 0xC0) {
              i += 2; // 2-byte UTF-8
          } else if ((c & 0xF0) == 0xE0) {
              i += 3; // 3-byte UTF-8
          } else if ((c & 0xF8) == 0xF0) {
              i += 4; // 4-byte UTF-8
          } else {
              ++i;
          }
        }
      }
  }

  return result;
}
