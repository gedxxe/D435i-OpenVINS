cmake_minimum_required(VERSION 3.22)
file(STRINGS "${CMAKE_CURRENT_LIST_DIR}/VERSION" OVRS_PROJECT_VERSION
     LIMIT_COUNT 1)
string(STRIP "${OVRS_PROJECT_VERSION}" OVRS_PROJECT_VERSION)
if(NOT OVRS_PROJECT_VERSION MATCHES "^[0-9]+\\.[0-9]+\\.[0-9]+$")
  message(FATAL_ERROR
    "VERSION must contain exactly one MAJOR.MINOR.PATCH semantic version")
endif()
project(ovrs_standalone VERSION "${OVRS_PROJECT_VERSION}" LANGUAGES CXX)
set(OVRS_PROJECT_VERSION_RESOLVED "${PROJECT_VERSION}" CACHE INTERNAL
    "Project version loaded from the repository VERSION file")

include(cmake/DependencyVersions.cmake)
include(cmake/SourceFingerprint.cmake)

# Content, rather than filesystem timestamps, identifies the project sources
# compiled into each executable. This prevents stale objects from being reused
# when the checkout and build directory live on a cross-platform NTFS mount.
ovrs_compute_source_fingerprint(
  "${CMAKE_CURRENT_SOURCE_DIR}" OVRS_SOURCE_FINGERPRINT)
set(OVRS_SOURCE_FINGERPRINT_RESOLVED "${OVRS_SOURCE_FINGERPRINT}"
    CACHE INTERNAL "Content fingerprint compiled into project executables")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)

if(WIN32)
  set(_ovrs_hardware_default OFF)
else()
  set(_ovrs_hardware_default ON)
endif()

option(OVRS_ENABLE_HARDWARE "Build librealsense2 capture support"
       ${_ovrs_hardware_default})
option(OVRS_ENABLE_OPENVINS
       "Build the pinned OpenVINS ${OVRS_OPENVINS_TAG} estimator wrapper"
       ${_ovrs_hardware_default})
option(OVRS_BUILD_TESTS "Build non-hardware tests" ON)
option(OVRS_ENABLE_SANITIZERS "Enable ASan and UBSan for project code" OFF)
set(OVRS_OPENVINS_PREFIX "" CACHE PATH
    "Repository-local OpenVINS install prefix created by build_ubuntu.sh")
set(OVRS_CERES_PREFIX
    "${CMAKE_CURRENT_SOURCE_DIR}/.deps/install/ceres" CACHE PATH
    "Repository-local Ceres ${OVRS_CERES_VERSION} install prefix")
set(OVRS_REALSENSE_PREFIX
    "${CMAKE_CURRENT_SOURCE_DIR}/.deps/install/librealsense" CACHE PATH
    "Repository-local patched librealsense ${OVRS_LIBREALSENSE_VERSION} prefix")

find_package(Threads REQUIRED)

function(ovrs_enable_project_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /permissive-)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endfunction()

function(ovrs_enable_project_sanitizers target)
  if(OVRS_ENABLE_SANITIZERS AND NOT MSVC)
    target_compile_options(${target} PRIVATE
      -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(${target} PRIVATE -fsanitize=address,undefined)
  endif()
endfunction()

set(_ovrs_opencv_components)
if(OVRS_ENABLE_HARDWARE)
  list(APPEND _ovrs_opencv_components core imgproc imgcodecs highgui)
endif()
if(OVRS_ENABLE_OPENVINS)
  list(APPEND _ovrs_opencv_components
    core imgproc features2d calib3d imgcodecs highgui)
endif()
if(_ovrs_opencv_components)
  list(REMOVE_DUPLICATES _ovrs_opencv_components)
  find_package(OpenCV 4 REQUIRED COMPONENTS ${_ovrs_opencv_components})
endif()

add_library(ovrs_core
  src/app_support.cpp
  src/calibration_validation.cpp
  src/capture_mode.cpp
  src/config.cpp
  src/imu_synchronizer.cpp
  src/measurement_dispatcher.cpp
  src/stereo_synchronizer.cpp
  src/timestamp_normalizer.cpp
  src/tracking_health.cpp
  src/trajectory.cpp
  src/trajectory_view.cpp
)
target_include_directories(ovrs_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(ovrs_core PUBLIC Threads::Threads)
target_compile_definitions(ovrs_core PUBLIC
  OVRS_PROJECT_VERSION="${PROJECT_VERSION}"
  OVRS_SOURCE_FINGERPRINT="${OVRS_SOURCE_FINGERPRINT}"
  OVRS_OPENVINS_TAG_VALUE="${OVRS_OPENVINS_TAG}"
  OVRS_OPENVINS_COMMIT_VALUE="${OVRS_OPENVINS_COMMIT}")
ovrs_enable_project_warnings(ovrs_core)
ovrs_enable_project_sanitizers(ovrs_core)

if(OVRS_ENABLE_HARDWARE)
  find_package(realsense2 ${OVRS_LIBREALSENSE_VERSION} EXACT CONFIG REQUIRED
    PATHS "${OVRS_REALSENSE_PREFIX}/lib/cmake/realsense2"
          "${OVRS_REALSENSE_PREFIX}/lib64/cmake/realsense2"
    NO_DEFAULT_PATH)
  if(TARGET realsense2::realsense2)
    set(_ovrs_realsense_target realsense2::realsense2)
  elseif(TARGET realsense2)
    set(_ovrs_realsense_target realsense2)
  elseif(realsense2_LIBRARY)
    set(_ovrs_realsense_target "${realsense2_LIBRARY}")
  else()
    message(FATAL_ERROR "librealsense2 package did not expose a link target")
  endif()
  add_library(ovrs_realsense src/realsense_source.cpp)
  target_include_directories(ovrs_realsense
    PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_include_directories(ovrs_realsense SYSTEM
    PRIVATE ${realsense2_INCLUDE_DIR})
  target_link_libraries(ovrs_realsense
    PUBLIC ovrs_core ${_ovrs_realsense_target})
  target_compile_definitions(ovrs_realsense PUBLIC OVRS_HAS_REALSENSE=1)
  target_compile_definitions(ovrs_realsense PUBLIC
    OVRS_REALSENSE_VERSION="${OVRS_LIBREALSENSE_VERSION}")
  ovrs_enable_project_warnings(ovrs_realsense)
  ovrs_enable_project_sanitizers(ovrs_realsense)

  add_library(ovrs_capture_preview src/stereo_capture_preview.cpp)
  target_include_directories(ovrs_capture_preview
    PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_include_directories(ovrs_capture_preview SYSTEM
    PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(ovrs_capture_preview
    PUBLIC ovrs_core PRIVATE ${OpenCV_LIBS})
  target_compile_definitions(ovrs_capture_preview PUBLIC
    OVRS_OPENCV_VERSION="${OpenCV_VERSION}")
  ovrs_enable_project_warnings(ovrs_capture_preview)
  ovrs_enable_project_sanitizers(ovrs_capture_preview)
endif()

if(OVRS_ENABLE_OPENVINS)
  if(NOT EXISTS "${OVRS_OPENVINS_PREFIX}/include/open_vins/core/VioManager.h")
    message(FATAL_ERROR
      "OVRS_OPENVINS_PREFIX does not contain the local OpenVINS install. "
      "Run scripts/build_ubuntu.sh or disable OVRS_ENABLE_OPENVINS. "
      "Expected OpenVINS ${OVRS_OPENVINS_TAG} at "
      "${OVRS_OPENVINS_COMMIT}.")
  endif()
  find_package(Eigen3 3.3 REQUIRED NO_MODULE)
  find_package(Boost REQUIRED COMPONENTS
    system filesystem thread date_time)
  find_package(Ceres ${OVRS_CERES_VERSION} EXACT CONFIG REQUIRED
    PATHS "${OVRS_CERES_PREFIX}/lib/cmake/Ceres"
          "${OVRS_CERES_PREFIX}/lib64/cmake/Ceres"
    NO_DEFAULT_PATH)
  find_library(OPENVINS_LIBRARY NAMES ov_msckf_lib
    PATHS "${OVRS_OPENVINS_PREFIX}/lib"
          "${OVRS_OPENVINS_PREFIX}/lib64"
    NO_DEFAULT_PATH REQUIRED)
  add_library(ovrs_openvins src/openvins_estimator.cpp)
  target_include_directories(ovrs_openvins
    PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_include_directories(ovrs_openvins SYSTEM
    PRIVATE "${OVRS_OPENVINS_PREFIX}/include/open_vins"
            ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(ovrs_openvins
    PUBLIC ovrs_core
    PRIVATE "${OPENVINS_LIBRARY}" Ceres::ceres Eigen3::Eigen
            ${OpenCV_LIBS} Boost::system Boost::filesystem
            Boost::thread Boost::date_time)
  target_compile_definitions(ovrs_openvins PUBLIC OVRS_HAS_OPENVINS=1)
  target_compile_definitions(ovrs_openvins PUBLIC
    OVRS_CERES_VERSION="${Ceres_VERSION}"
    OVRS_OPENCV_VERSION="${OpenCV_VERSION}")
  ovrs_enable_project_warnings(ovrs_openvins)
  ovrs_enable_project_sanitizers(ovrs_openvins)

  add_library(ovrs_viewer src/live_viewer.cpp)
  target_include_directories(ovrs_viewer
    PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_include_directories(ovrs_viewer SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(ovrs_viewer PUBLIC ovrs_core PRIVATE ${OpenCV_LIBS})
  ovrs_enable_project_warnings(ovrs_viewer)
  ovrs_enable_project_sanitizers(ovrs_viewer)
endif()

foreach(app IN ITEMS ovrs_inspect ovrs_record ovrs_live ovrs_replay)
  add_executable(${app} "apps/${app}.cpp")
  target_link_libraries(${app} PRIVATE ovrs_core)
  if(OVRS_ENABLE_HARDWARE AND
     (app STREQUAL "ovrs_inspect" OR app STREQUAL "ovrs_record" OR
      app STREQUAL "ovrs_live"))
    target_link_libraries(${app} PRIVATE ovrs_realsense)
  endif()
  if(OVRS_ENABLE_OPENVINS AND
     (app STREQUAL "ovrs_live" OR app STREQUAL "ovrs_replay"))
    target_link_libraries(${app} PRIVATE ovrs_openvins ovrs_viewer)
  endif()
  if(OVRS_ENABLE_HARDWARE AND app STREQUAL "ovrs_record")
    target_include_directories(${app} SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${app} PRIVATE ovrs_capture_preview ${OpenCV_LIBS})
  endif()
  if(OVRS_ENABLE_OPENVINS AND app STREQUAL "ovrs_replay")
    target_include_directories(${app} SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${app} PRIVATE ${OpenCV_LIBS})
  endif()
  ovrs_enable_project_warnings(${app})
  ovrs_enable_project_sanitizers(${app})
endforeach()

if(OVRS_BUILD_TESTS)
  enable_testing()
  add_executable(ovrs_core_tests tests/core_tests.cpp)
  target_include_directories(ovrs_core_tests PRIVATE tests)
  target_link_libraries(ovrs_core_tests PRIVATE ovrs_core)
  ovrs_enable_project_warnings(ovrs_core_tests)
  ovrs_enable_project_sanitizers(ovrs_core_tests)
  add_test(NAME ovrs_core_tests COMMAND ovrs_core_tests)

  add_executable(ovrs_mock_replay_test tests/mock_replay_test.cpp)
  target_include_directories(ovrs_mock_replay_test PRIVATE tests)
  target_link_libraries(ovrs_mock_replay_test PRIVATE ovrs_core)
  ovrs_enable_project_warnings(ovrs_mock_replay_test)
  ovrs_enable_project_sanitizers(ovrs_mock_replay_test)
  add_test(NAME ovrs_mock_replay_test COMMAND ovrs_mock_replay_test)

  if(NOT Python3_EXECUTABLE)
    if(WIN32)
      set(OVRS_LOCAL_PYTHON
          "${CMAKE_CURRENT_SOURCE_DIR}/.venv/Scripts/python.exe")
    else()
      set(OVRS_LOCAL_PYTHON
          "${CMAKE_CURRENT_SOURCE_DIR}/.venv/bin/python")
    endif()
    if(EXISTS "${OVRS_LOCAL_PYTHON}")
      set(Python3_EXECUTABLE "${OVRS_LOCAL_PYTHON}"
          CACHE FILEPATH "Python interpreter used by repository tests")
    endif()
  endif()
  find_package(Python3 3.11 QUIET COMPONENTS Interpreter)
  if(Python3_Interpreter_FOUND)
    add_test(
      NAME ovrs_calibration_scripts_test
      COMMAND "${Python3_EXECUTABLE}"
              "${CMAKE_CURRENT_SOURCE_DIR}/tests/calibration_scripts_test.py")
  endif()

  find_program(OVRS_BASH_EXECUTABLE bash)
  if(UNIX AND OVRS_BASH_EXECUTABLE)
    add_test(
      NAME ovrs_repository_checks_test
      COMMAND "${OVRS_BASH_EXECUTABLE}"
              "${CMAKE_CURRENT_SOURCE_DIR}/tests/repository_checks_test.sh")
  endif()
endif()

install(TARGETS ovrs_inspect ovrs_record ovrs_live ovrs_replay
        RUNTIME DESTINATION bin)
