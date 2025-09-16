# Copyright (c) 2025，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(hobot_xlm)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Werror -Wl,-unresolved-symbols=ignore-in-shared-libs")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-function -Wno-unused-parameter -Wno-unused-variable")

set(CMAKE_BUILD_TYPE ${build_type})

# debug
set(CMAKE_CXX_FLAGS_DEBUG " -g -O0 -ggdb -pipe -fPIC ")
set(CMAKE_C_FLAGS_DEBUG " -g -O0 -ggdb -pipe -fPIC ")
# release
set(CMAKE_CXX_FLAGS_RELEASE "  -O3 -pipe -fPIC ")
set(CMAKE_C_FLAGS_RELEASE " -O3 -pipe -fPIC ")

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ai_msgs REQUIRED)

if(PLATFORM_S100)
  message("build platform S100")
  add_definitions(-DPLATFORM_S100)
  set(PREFIX_PATH s100)
else()
  message("invalid platform, build platform S100 default")
  set(PREFIX_PATH s100)
endif()

message("CMAKE_SYSROOT is " ${CMAKE_SYSROOT})
include_directories(
  include
  ${PROJECT_SOURCE_DIR}
  ${CMAKE_SYSROOT}/usr/include/
  ${CMAKE_SYSROOT}/usr/include/hobot/
  ${CMAKE_SYSROOT}/usr/hobot/include/
  ${CMAKE_SYSROOT}/usr/include/opencv/
)

message("CMAKE_CURRENT_SOURCE_DIR is " ${CMAKE_CURRENT_SOURCE_DIR})
message("CMAKE_SOURCE_DIR is " ${CMAKE_SOURCE_DIR})
link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/
  ${CMAKE_SYSROOT}/usr/lib/hbbpu/
  ${CMAKE_SYSROOT}/usr/hobot/lib/
  ${CMAKE_SYSROOT}/usr/lib/hbmedia/
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/xlm_node.cpp
  src/util.cpp
)

target_link_libraries(${PROJECT_NAME} xlm)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  ai_msgs
)

# Install executables
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/
  DESTINATION lib/${PROJECT_NAME}/config/
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/lib/
  DESTINATION lib/${PROJECT_NAME}/lib/
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()