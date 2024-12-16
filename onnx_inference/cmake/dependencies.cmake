# Create dependencies macro
macro(onnx_dependencies)
    include(FetchContent)

    if(UNIX AND NOT APPLE)
        FetchContent_Declare(
            onnxruntime
            DOWNLOAD_EXTRACT_TIMESTAMP FALSE
            URL https://github.com/microsoft/onnxruntime/releases/download/v1.20.1/onnxruntime-linux-x64-1.20.1.tgz
        )
        set(LIBRARY_NAME "libonnxruntime.so")
    elseif(APPLE)
        FetchContent_Declare(
            onnxruntime
            DOWNLOAD_EXTRACT_TIMESTAMP FALSE
            URL https://github.com/microsoft/onnxruntime/releases/download/v1.20.1/onnxruntime-osx-universal2-1.20.1.tgz
        )
        set(LIBRARY_NAME "libonnxruntime.dylib")
    endif()

    FetchContent_MakeAvailable(onnxruntime)

    # Library path: ${onnxruntime_SOURCE_DIR}/lib/libonnxruntime.so
    # Include path: ${onnxruntime_SOURCE_DIR}/include/
    add_library(onnxruntime SHARED IMPORTED)
    set_target_properties(onnxruntime PROPERTIES
        IMPORTED_LOCATION ${onnxruntime_SOURCE_DIR}/lib/${LIBRARY_NAME}
        INTERFACE_INCLUDE_DIRECTORIES ${onnxruntime_SOURCE_DIR}/include/
    )
    # Install the library
    file(GLOB ONNXRUNTIME_LIBS "${onnxruntime_SOURCE_DIR}/lib/libonnxruntime.*")
    install(FILES ${ONNXRUNTIME_LIBS} DESTINATION lib)
endmacro()