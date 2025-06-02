cmake_minimum_required(VERSION 3.21)

project("onnx_inference"
        VERSION 0.1
        DESCRIPTION "Inference with ONNX models"
        LANGUAGES CXX)

if(CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
    message(FATAL_ERROR "Source and build directories cannot be the same.")
endif()

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

include(cmake/dependencies.cmake)
onnx_dependencies()

# Set the RPATH to the lib directory, so that the shared library can be found
# it is necessary to set the RPATH before building the targets
list(APPEND CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

add_library(onnx_actor SHARED src/cpp/onnx_actor.cpp)
target_include_directories(onnx_actor PUBLIC include)
target_link_libraries(onnx_actor PUBLIC onnxruntime)

add_executable(onnx_inference src/cpp/main.cpp)
target_link_libraries(onnx_inference PUBLIC onnx_actor ament_index_cpp::ament_index_cpp)

# Make data files available in the build directory
file(COPY data DESTINATION ${CMAKE_BINARY_DIR})

# Install the shared library and the executable
install(TARGETS onnx_actor
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(TARGETS onnx_inference
        RUNTIME DESTINATION bin)

# Install the header files
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY data/ DESTINATION share/${PROJECT_NAME}/data)

ament_package()