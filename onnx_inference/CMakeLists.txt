cmake_minimum_required(VERSION 3.21)

project("onnx-actor"
        VERSION 0.1
        DESCRIPTION "Inference with ONNX models"
        LANGUAGES CXX)

if(CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
    message(FATAL_ERROR "Source and build directories cannot be the same.")
endif()

set(CMAKE_CXX_STANDARD 17)

include(cmake/dependencies.cmake)
onnx_dependencies()

add_library(onnx-actor SHARED src/cpp/onnx_actor.cpp)
target_include_directories(onnx-actor PUBLIC include)
target_link_libraries(onnx-actor onnxruntime)

add_executable(onnx-inference src/cpp/main.cpp)
target_link_libraries(onnx-inference onnx-actor)

# Make data files available in the build directory
file(COPY data DESTINATION ${CMAKE_BINARY_DIR})