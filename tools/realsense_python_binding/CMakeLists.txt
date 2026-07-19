cmake_minimum_required(VERSION 3.16)
project(k1_pyrealsense2_binding LANGUAGES CXX)

if(NOT LIBREALSENSE_SOURCE_DIR)
  message(FATAL_ERROR "LIBREALSENSE_SOURCE_DIR is required")
endif()
if(NOT PYBIND11_JSON_INCLUDE_DIR)
  message(FATAL_ERROR "PYBIND11_JSON_INCLUDE_DIR is required")
endif()

find_package(Python3 3.12 EXACT COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
find_package(Threads REQUIRED)

find_library(
  REALSENSE2_LIBRARY
  NAMES realsense2
  PATHS /opt/ros/humble/lib/riscv64-linux-gnu
  NO_DEFAULT_PATH
  REQUIRED
)
find_library(
  RSUTILS_LIBRARY
  NAMES rsutils
  PATHS /opt/ros/humble/lib/riscv64-linux-gnu
  NO_DEFAULT_PATH
  REQUIRED
)

set(PYRS_DIR "${LIBREALSENSE_SOURCE_DIR}/wrappers/python")
set(PYRS_SOURCES
  "${PYRS_DIR}/pyrealsense2.cpp"
  "${PYRS_DIR}/c_files.cpp"
  "${PYRS_DIR}/pyrs_advanced_mode.cpp"
  "${PYRS_DIR}/pyrs_context.cpp"
  "${PYRS_DIR}/pyrs_device.cpp"
  "${PYRS_DIR}/pyrs_export.cpp"
  "${PYRS_DIR}/pyrs_frame.cpp"
  "${PYRS_DIR}/pyrs_internal.cpp"
  "${PYRS_DIR}/pyrs_options.cpp"
  "${PYRS_DIR}/pyrs_pipeline.cpp"
  "${PYRS_DIR}/pyrs_processing.cpp"
  "${PYRS_DIR}/pyrs_record_playback.cpp"
  "${PYRS_DIR}/pyrs_sensor.cpp"
  "${PYRS_DIR}/pyrs_types.cpp"
  "${PYRS_DIR}/pyrsutil.cpp"
  "${LIBREALSENSE_SOURCE_DIR}/common/metadata-helper.cpp"
)

pybind11_add_module(pyrealsense2 MODULE NO_EXTRAS ${PYRS_SOURCES})
target_compile_features(pyrealsense2 PRIVATE cxx_std_14)
target_compile_definitions(
  pyrealsense2
  PRIVATE
    BUILD_EASYLOGGINGPP
    BUILD_SHARED_LIBS
    COM_MULTITHREADED
    EASYLOGGINGPP_ASYNC
    ELPP_NO_DEFAULT_LOG_FILE
    ELPP_THREAD_SAFE
    HWM_OVER_XU
    RS2_USE_V4L2_BACKEND
    UNICODE
)
target_compile_options(
  pyrealsense2
  PRIVATE
    -O2
    -ftree-vectorize
    -Wno-missing-field-initializers
    -Wno-switch
    -Wno-multichar
)
target_include_directories(
  pyrealsense2
  PRIVATE
    "${LIBREALSENSE_SOURCE_DIR}"
    "${LIBREALSENSE_SOURCE_DIR}/src"
    "${LIBREALSENSE_SOURCE_DIR}/include"
    "${LIBREALSENSE_SOURCE_DIR}/third-party/rsutils/include"
    "${PYBIND11_JSON_INCLUDE_DIR}"
)
target_link_libraries(
  pyrealsense2
  PRIVATE
    "${REALSENSE2_LIBRARY}"
    "${RSUTILS_LIBRARY}"
    Threads::Threads
)
set_target_properties(
  pyrealsense2
  PROPERTIES
    BUILD_RPATH "/opt/ros/humble/lib/riscv64-linux-gnu"
    INSTALL_RPATH "/opt/ros/humble/lib/riscv64-linux-gnu"
)

install(TARGETS pyrealsense2 LIBRARY DESTINATION lib/python3.12/site-packages)
