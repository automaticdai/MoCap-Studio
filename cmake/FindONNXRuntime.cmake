# FindONNXRuntime.cmake
# Finds ONNX Runtime C++ library
#
# Sets:
#   ONNXRuntime_FOUND
#   ONNXRuntime_INCLUDE_DIRS
#   ONNXRuntime_LIBRARIES
#
# Set ONNXRUNTIME_ROOT (CMake var or env var) to point at a custom install.
# Versioned prebuilt archives extracted under /opt or /usr/local
# (e.g. /opt/onnxruntime-linux-x64-gpu-1.17.0) are discovered automatically.

file(GLOB _onnxruntime_versioned_dirs
    /opt/onnxruntime*
    /usr/local/onnxruntime*
)

set(_onnxruntime_hints
    ${ONNXRUNTIME_ROOT}
    $ENV{ONNXRUNTIME_ROOT}
    ${_onnxruntime_versioned_dirs}
    /opt/onnxruntime
    /usr/local
    /usr
)

find_path(ONNXRuntime_INCLUDE_DIR
    NAMES onnxruntime_cxx_api.h
    PATH_SUFFIXES onnxruntime/core/session include
    HINTS ${_onnxruntime_hints}
)

find_library(ONNXRuntime_LIBRARY
    NAMES onnxruntime
    PATH_SUFFIXES lib
    HINTS ${_onnxruntime_hints}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ONNXRuntime
    REQUIRED_VARS ONNXRuntime_LIBRARY ONNXRuntime_INCLUDE_DIR
)

if(ONNXRuntime_FOUND)
    set(ONNXRuntime_INCLUDE_DIRS ${ONNXRuntime_INCLUDE_DIR})
    set(ONNXRuntime_LIBRARIES ${ONNXRuntime_LIBRARY})
    if(NOT TARGET ONNXRuntime::ONNXRuntime)
        add_library(ONNXRuntime::ONNXRuntime UNKNOWN IMPORTED)
        set_target_properties(ONNXRuntime::ONNXRuntime PROPERTIES
            IMPORTED_LOCATION "${ONNXRuntime_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${ONNXRuntime_INCLUDE_DIR}"
        )
    endif()
endif()
