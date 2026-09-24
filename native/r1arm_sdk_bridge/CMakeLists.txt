cmake_minimum_required(VERSION 3.16)
project(r1arm_sdk_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(R1ARM_SDK_ROOT_DEFAULT "/home/ycb410/nt/R1_arm_SDK_x86/r1_arm")
if(DEFINED ENV{R1ARM_SDK_ROOT} AND NOT "$ENV{R1ARM_SDK_ROOT}" STREQUAL "")
  set(R1ARM_SDK_ROOT_DEFAULT "$ENV{R1ARM_SDK_ROOT}")
endif()

set(
  R1ARM_SDK_ROOT
  "${R1ARM_SDK_ROOT_DEFAULT}"
  CACHE PATH
  "Path to the x86 R1 arm SDK root"
)

set(R1ARM_BUS_HEADER "${R1ARM_SDK_ROOT}/include/arm_motor_bus.h")
set(R1ARM_BUS_SOURCE "${R1ARM_SDK_ROOT}/src/arm_motor_bus.cpp")
set(R1ARM_MOTOR_INCLUDE "${R1ARM_SDK_ROOT}/thirdparty/motor_controller/include")
set(R1ARM_MOTOR_LIB_DIR "${R1ARM_SDK_ROOT}/thirdparty/motor_controller/lib")

foreach(required_path
    "${R1ARM_BUS_HEADER}"
    "${R1ARM_BUS_SOURCE}"
    "${R1ARM_MOTOR_INCLUDE}")
  if(NOT EXISTS "${required_path}")
    message(FATAL_ERROR "R1Arm SDK file/directory not found: ${required_path}")
  endif()
endforeach()

find_package(Threads REQUIRED)
find_library(
  R1ARM_MOTOR_CONTROLLER
  NAMES motor_controller
  PATHS "${R1ARM_MOTOR_LIB_DIR}"
  NO_DEFAULT_PATH
  REQUIRED
)

add_library(r1arm_sdk_bridge SHARED
  r1arm_sdk_bridge.cpp
  "${R1ARM_BUS_SOURCE}"
)
target_include_directories(r1arm_sdk_bridge PRIVATE
  "${R1ARM_SDK_ROOT}/include"
  "${R1ARM_MOTOR_INCLUDE}"
)
target_link_libraries(r1arm_sdk_bridge PRIVATE
  "${R1ARM_MOTOR_CONTROLLER}"
  Threads::Threads
)
set_target_properties(r1arm_sdk_bridge PROPERTIES
  BUILD_RPATH "${R1ARM_MOTOR_LIB_DIR}"
  INSTALL_RPATH "${R1ARM_MOTOR_LIB_DIR}"
)
