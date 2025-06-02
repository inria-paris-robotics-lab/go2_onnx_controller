cmake_minimum_required(VERSION 3.21)

project(onnx_inference
  VERSION 0.1
  DESCRIPTION "Inference with ONNX models"
)

if(CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
  message(FATAL_ERROR "Do not build in-source. Please use a separate build directory.")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

include(cmake/dependencies.cmake)
onnx_dependencies()

# Assume onnx_dependencies() sets ONNXRUNTIME_LIBRARIES and ONNXRUNTIME_INCLUDE_DIRS
add_library(onnx_actor SHARED
  src/cpp/onnx_actor.cpp
)

target_include_directories(onnx_actor PUBLIC
  ${ONNXRUNTIME_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(onnx_actor PUBLIC onnxruntime)

add_executable(onnx_inference
  src/cpp/main.cpp
)

target_link_libraries(onnx_inference
  onnx_actor
)

ament_target_dependencies(onnx_inference
  ament_index_cpp
)

install(TARGETS
  onnx_actor
  EXPORT export_onnx_actor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  onnx_inference
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

# Export the library interface for use by downstream packages
ament_export_targets(export_onnx_actor HAS_LIBRARY_TARGET)

ament_package()
