# Create dependencies macro
macro(onnx_dependencies)
    include(FetchContent)

    FetchContent_Declare(
        onnxruntime
        URL https://github.com/microsoft/onnxruntime/releases/download/v1.20.1/onnxruntime-linux-x64-1.20.1.tgz
        DOWNLOAD_EXTRACT_TIMESTAMP FALSE
    )

    FetchContent_MakeAvailable(onnxruntime)

    # Library path: ${onnxruntime_SOURCE_DIR}/lib/libonnxruntime.so
    # Include path: ${onnxruntime_SOURCE_DIR}/include/
    add_library(onnxruntime SHARED IMPORTED)
    set_target_properties(onnxruntime PROPERTIES
        IMPORTED_LOCATION ${onnxruntime_SOURCE_DIR}/lib/libonnxruntime.so
        INTERFACE_INCLUDE_DIRECTORIES ${onnxruntime_SOURCE_DIR}/include/
    )
endmacro()