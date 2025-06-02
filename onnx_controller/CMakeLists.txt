cmake_minimum_required(VERSION 3.8)
project(onnx_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)

# Include MCAP
include(FetchContent)
FetchContent_Declare(
  mcap_builder
  GIT_REPOSITORY https://github.com/olympus-robotics/mcap_builder.git
  GIT_TAG main
)
FetchContent_MakeAvailable(mcap_builder)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_go REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(onnx_interfaces REQUIRED)

# create your targets
add_executable(controller
  src/controller.cpp
  src/motor_crc.cpp
  src/robot_interface.cpp
)
# TODO: Why do we need to link onnxruntime here when we already linked it in onnx_actor?
target_link_libraries(controller onnx_actor onnxruntime mcap)

ament_target_dependencies(controller rclcpp unitree_go sensor_msgs onnx_interfaces)
target_include_directories(controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/onnx_controller>) # for headers when building)

install(TARGETS
  controller
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

