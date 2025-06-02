# Create dependencies macro
macro(onnx_dependencies)
    include(FetchContent)

    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
        set(architecture "aarch64")
    elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
        set(architecture "x64")
    else()
        message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    endif()

    if(UNIX AND NOT APPLE)
        FetchContent_Declare(
            onnxruntime
            DOWNLOAD_EXTRACT_TIMESTAMP FALSE
            URL https://github.com/microsoft/onnxruntime/releases/download/v1.20.1/onnxruntime-linux-${architecture}-1.20.1.tgz
        )
        set(LIBRARY_NAME "libonnxruntime.so")
    elseif(APPLE)
        FetchContent_Declare(
            onnxruntime
            DOWNLOAD_EXTRACT_TIMESTAMP FALSE
            URL https://github.com/microsoft/onnxruntime/releases/download/v1.20.1/onnxruntime-osx-universal2-1.20.1.tgz
        )
        set(LIBRARY_NAME "libonnxruntime.dylib")
    else()
        message(FATAL_ERROR "Unsupported platform")
    endif()

    FetchContent_MakeAvailable(onnxruntime)

    if(NOT APPLE)
        file(GLOB ONNXRUNTIME_LIBS ${onnxruntime_SOURCE_DIR}/lib/libonnxruntime.so*)
    else()
        file(GLOB ONNXRUNTIME_LIBS ${onnxruntime_SOURCE_DIR}/lib/*.dylib)
    endif()

    message(STATUS "ONNXRUNTIME_LIBS: ${ONNXRUNTIME_LIBS}")

    # Library path: ${onnxruntime_SOURCE_DIR}/lib/libonnxruntime.so
    # Include path: ${onnxruntime_SOURCE_DIR}/include/
    add_library(onnxruntime SHARED IMPORTED)
    set_target_properties(onnxruntime PROPERTIES
        IMPORTED_LOCATION ${onnxruntime_SOURCE_DIR}/lib/${LIBRARY_NAME}
        INTERFACE_INCLUDE_DIRECTORIES ${onnxruntime_SOURCE_DIR}/include/
    )
    # Install the library
    install(FILES ${ONNXRUNTIME_LIBS} DESTINATION lib)

    # Install the headers
    install(DIRECTORY ${onnxruntime_SOURCE_DIR}/include/ DESTINATION include)
endmacro()
