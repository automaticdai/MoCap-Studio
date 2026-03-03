set(MOCAP_CORE_SOURCES
    core/config.cpp
    core/skeleton_definition.cpp
    core/camera_intrinsics.cpp
)

set(MOCAP_CAPTURE_SOURCES
    capture/usb_camera_source.cpp
    capture/ip_camera_source.cpp
    capture/video_file_camera_source.cpp
    capture/frame_broker.cpp
)

set(MOCAP_POSE_SOURCES
    pose/onnx_pose_estimator.cpp
    pose/person_tracker.cpp
)

set(MOCAP_TRIANGULATION_SOURCES
    triangulation/triangulator.cpp
    triangulation/temporal_filter.cpp
)

set(MOCAP_SKELETON_SOURCES
    skeleton/skeleton_solver.cpp
)

set(MOCAP_STORAGE_SOURCES
    storage/session_manager.cpp
    storage/binary_io.cpp
    storage/exporters/csv_exporter.cpp
    storage/exporters/json_exporter.cpp
    storage/exporters/c3d_exporter.cpp
    storage/exporters/bvh_exporter.cpp
    storage/exporters/fbx_exporter.cpp
    storage/exporters/usd_exporter.cpp
)

set(MOCAP_GUI_SOURCES
    gui/main_window.cpp
    gui/camera_feed_widget.cpp
    gui/mocap_canvas.cpp
    gui/timeline_widget.cpp
    gui/inspector_panel.cpp
    gui/session_dialog.cpp
    gui/calibration_wizard.cpp
    gui/export_dialog.cpp
)

# Optional Blackmagic source
if(MOCAP_ENABLE_BLACKMAGIC)
    list(APPEND MOCAP_CAPTURE_SOURCES capture/blackmagic_camera_source.cpp)
endif()

# Core library (everything except main.cpp)
add_library(mocap_lib STATIC
    ${MOCAP_CORE_SOURCES}
    ${MOCAP_CAPTURE_SOURCES}
    ${MOCAP_POSE_SOURCES}
    ${MOCAP_TRIANGULATION_SOURCES}
    ${MOCAP_SKELETON_SOURCES}
    ${MOCAP_STORAGE_SOURCES}
    ${MOCAP_GUI_SOURCES}
)

target_include_directories(mocap_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(mocap_lib PUBLIC
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::OpenGLWidgets
    ${OpenCV_LIBS}
    ONNXRuntime::ONNXRuntime
    Eigen3::Eigen
    spdlog::spdlog
    yaml-cpp::yaml-cpp
    nlohmann_json::nlohmann_json
    ezc3d
)

if(assimp_FOUND)
    target_link_libraries(mocap_lib PUBLIC assimp::assimp)
    target_compile_definitions(mocap_lib PUBLIC MOCAP_HAS_ASSIMP)
endif()

if(MOCAP_ENABLE_BLACKMAGIC)
    target_compile_definitions(mocap_lib PUBLIC MOCAP_HAS_BLACKMAGIC)
endif()

# Executable
add_executable(MoCapStudio main.cpp)
target_link_libraries(MoCapStudio PRIVATE mocap_lib)
