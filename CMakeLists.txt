cmake_minimum_required(VERSION 3.22)
project(MoCapStudio VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

option(MOCAP_BUILD_TESTS "Build tests" ON)
option(MOCAP_ENABLE_BLACKMAGIC "Enable Blackmagic DeckLink support" OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Qt6
find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGL OpenGLWidgets)

# OpenCV
find_package(OpenCV 4.0 REQUIRED COMPONENTS core imgproc imgcodecs videoio calib3d)

# ONNX Runtime
find_package(ONNXRuntime REQUIRED)

# Eigen (header-only, try system first, then FetchContent)
find_package(Eigen3 3.4 QUIET)
if(NOT Eigen3_FOUND)
    include(FetchContent)
    FetchContent_Declare(eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
    )
    FetchContent_MakeAvailable(eigen)
endif()

# spdlog (always use FetchContent to avoid system fmt incompatibilities)
include(FetchContent)
FetchContent_Declare(spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.14.1
)
set(SPDLOG_FMT_EXTERNAL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(spdlog)

# yaml-cpp
find_package(yaml-cpp 0.7 QUIET)
if(NOT yaml-cpp_FOUND)
    include(FetchContent)
    FetchContent_Declare(yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG 0.8.0
    )
    FetchContent_MakeAvailable(yaml-cpp)
endif()

# nlohmann/json (header-only)
include(FetchContent)
FetchContent_Declare(json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(json)

# ezc3d
FetchContent_Declare(ezc3d
    GIT_REPOSITORY https://github.com/pyomeca/ezc3d.git
    GIT_TAG Release_1.5.8
)
FetchContent_MakeAvailable(ezc3d)

# Assimp (optional, for FBX export)
find_package(assimp QUIET)

add_subdirectory(src)

if(MOCAP_BUILD_TESTS)
    enable_testing()
    FetchContent_Declare(googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(BUILD_SHARED_LIBS OFF)
    FetchContent_MakeAvailable(googletest)
    set(BUILD_SHARED_LIBS ON)
    add_subdirectory(tests)
endif()
