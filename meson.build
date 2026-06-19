project('MoCapStudio', 'cpp',
  version : '0.1.0',
  meson_version : '>=0.57.0',
  default_options : ['cpp_std=c++17', 'buildtype=release', 'warning_level=2'])

cpp = meson.get_compiler('cpp')

# ---------------------------------------------------------------------------
# Qt6 (Widgets + OpenGL + OpenGLWidgets), with moc for Q_OBJECT headers
# ---------------------------------------------------------------------------
qt6 = import('qt6')
# Core and Gui are listed explicitly: unlike CMake's imported targets, Meson's
# qmake-based Qt dependency does not add include dirs for transitive modules.
qt6_dep = dependency('qt6', modules : ['Core', 'Gui', 'Widgets', 'OpenGL', 'OpenGLWidgets'])

# ---------------------------------------------------------------------------
# OpenCV
# ---------------------------------------------------------------------------
opencv_dep = dependency('opencv4', version : '>=4.0')

# ---------------------------------------------------------------------------
# ONNX Runtime
# Replicates the old cmake/FindONNXRuntime.cmake auto-discovery: honour an
# explicit -Donnxruntime_root, otherwise probe /opt/onnxruntime* and
# /usr/local/onnxruntime*, otherwise fall back to the default system search.
# ---------------------------------------------------------------------------
ort_root = get_option('onnxruntime_root')
if ort_root == ''
  ort_probe = run_command('sh', '-c',
    'ls -d /opt/onnxruntime* /usr/local/onnxruntime* 2>/dev/null | head -1',
    check : false)
  ort_root = ort_probe.stdout().strip()
endif
if ort_root != ''
  ort_libdirs = [ort_root / 'lib']
  ort_incdirs = include_directories(ort_root / 'include')
  message('Using ONNX Runtime at ' + ort_root)
else
  ort_libdirs = []
  ort_incdirs = include_directories()
endif
ort_lib = cpp.find_library('onnxruntime', dirs : ort_libdirs, required : true)
onnxruntime_dep = declare_dependency(
  dependencies : ort_lib,
  include_directories : ort_incdirs)

# ---------------------------------------------------------------------------
# Auto-fetched / system dependencies (system first, subproject fallback)
# ---------------------------------------------------------------------------
eigen_dep  = dependency('eigen3', version : '>=3.4')   # system, or eigen.wrap
yamlcpp_dep = dependency('yaml-cpp')                   # system, or yaml-cpp.wrap
json_dep   = dependency('nlohmann_json')               # nlohmann_json.wrap (header-only)

# spdlog is always built from the subproject with its bundled fmt, mirroring the
# CMake build which used FetchContent to avoid system fmt incompatibilities.
spdlog_dep = subproject('spdlog').get_variable('spdlog_dep')

# ezc3d has no WrapDB entry and ships a CMake build; consume it via the cmake
# module (system first if a config/pkg-config happens to be installed).
cmake = import('cmake')
ezc3d_dep = dependency('ezc3d', required : false)
if not ezc3d_dep.found()
  ezc3d_opts = cmake.subproject_options()
  ezc3d_opts.add_cmake_defines({'BUILD_EXAMPLE': false})
  ezc3d_proj = cmake.subproject('ezc3d', options : ezc3d_opts)
  ezc3d_dep = ezc3d_proj.dependency('ezc3d')
endif

# Optional Assimp (FBX export). Compiled in only when present, like the CMake build.
assimp_dep = dependency('assimp', required : false)

# ---------------------------------------------------------------------------
# Conditional compile definitions
# ---------------------------------------------------------------------------
if assimp_dep.found()
  add_project_arguments('-DMOCAP_HAS_ASSIMP', language : 'cpp')
endif
if get_option('blackmagic')
  add_project_arguments('-DMOCAP_HAS_BLACKMAGIC', language : 'cpp')
endif
if get_option('directml')
  add_project_arguments('-DMOCAP_USE_DIRECTML', language : 'cpp')
endif

# ---------------------------------------------------------------------------
# Sources
# ---------------------------------------------------------------------------
inc = include_directories('src')

moc_headers = files(
  'src/capture/frame_broker.h',
  'src/gui/mocap_canvas.h',
  'src/gui/timeline_widget.h',
  'src/gui/session_dialog.h',
  'src/gui/export_dialog.h',
  'src/gui/camera_management_dialog.h',
  'src/gui/settings_dialog.h',
  'src/gui/main_window.h',
  'src/gui/camera_feed_widget.h',
  'src/gui/inspector_panel.h',
  'src/gui/camera_edit_dialog.h',
  'src/gui/calibration_wizard.h',
)
moc_files = qt6.preprocess(moc_headers : moc_headers)

mocap_sources = files(
  # core
  'src/core/config.cpp',
  'src/core/skeleton_definition.cpp',
  'src/core/camera_intrinsics.cpp',
  'src/core/intrinsics_calibrator.cpp',
  # capture
  'src/capture/usb_camera_source.cpp',
  'src/capture/ip_camera_source.cpp',
  'src/capture/video_file_camera_source.cpp',
  'src/capture/frame_broker.cpp',
  # pose
  'src/pose/onnx_pose_estimator.cpp',
  'src/pose/person_tracker.cpp',
  # triangulation
  'src/triangulation/triangulator.cpp',
  'src/triangulation/temporal_filter.cpp',
  # skeleton
  'src/skeleton/skeleton_solver.cpp',
  # storage
  'src/storage/session_manager.cpp',
  'src/storage/binary_io.cpp',
  'src/storage/exporters/csv_exporter.cpp',
  'src/storage/exporters/json_exporter.cpp',
  'src/storage/exporters/c3d_exporter.cpp',
  'src/storage/exporters/bvh_exporter.cpp',
  'src/storage/exporters/fbx_exporter.cpp',
  'src/storage/exporters/usd_exporter.cpp',
  # gui
  'src/gui/main_window.cpp',
  'src/gui/camera_feed_widget.cpp',
  'src/gui/mocap_canvas.cpp',
  'src/gui/timeline_widget.cpp',
  'src/gui/inspector_panel.cpp',
  'src/gui/session_dialog.cpp',
  'src/gui/calibration_wizard.cpp',
  'src/gui/export_dialog.cpp',
  'src/gui/camera_edit_dialog.cpp',
  'src/gui/camera_management_dialog.cpp',
  'src/gui/settings_dialog.cpp',
)

if get_option('blackmagic')
  mocap_sources += files('src/capture/blackmagic_camera_source.cpp')
endif

mocap_deps = [
  qt6_dep,
  opencv_dep,
  onnxruntime_dep,
  eigen_dep,
  spdlog_dep,
  yamlcpp_dep,
  json_dep,
  ezc3d_dep,
]
if assimp_dep.found()
  mocap_deps += assimp_dep
endif

# Core library (everything except main.cpp)
mocap_lib = static_library('mocap',
  mocap_sources + moc_files,
  include_directories : inc,
  dependencies : mocap_deps)

mocap_dep = declare_dependency(
  link_with : mocap_lib,
  include_directories : inc,
  dependencies : mocap_deps)

# Executable
executable('MoCapStudio',
  'src/main.cpp',
  dependencies : mocap_dep,
  install : true)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if get_option('tests')
  gtest_proj = subproject('gtest')
  gtest_main_dep = gtest_proj.get_variable('gtest_main_dep')
  gmock_dep = gtest_proj.get_variable('gmock_dep')

  test_names = [
    'test_types',
    'test_triangulation',
    'test_skeleton_solver',
    'test_temporal_filter',
    'test_binary_io',
    'test_config',
    'test_skeleton_definition',
    'test_bvh_exporter',
    'test_csv_json_exporter',
    'test_person_tracker',
    'test_intrinsics_calibrator',
  ]
  foreach t : test_names
    exe = executable(t,
      files('tests/' + t + '.cpp'),
      dependencies : [mocap_dep, gtest_main_dep, gmock_dep])
    test(t, exe, protocol : 'gtest')
  endforeach
endif
