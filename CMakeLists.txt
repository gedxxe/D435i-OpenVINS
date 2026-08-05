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
option(OVRS_ENABLE_ORBSLAM3
       "Build the experimental pinned ORB-SLAM3 live adapter" OFF)
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
set(OVRS_ORBSLAM3_SOURCE
    "${CMAKE_CURRENT_SOURCE_DIR}/.deps/src/orb_slam3" CACHE PATH
    "Pinned and reviewed ORB-SLAM3 source checkout")
set(OVRS_ORBSLAM3_LIBRARY
    "${OVRS_ORBSLAM3_SOURCE}/lib/libORB_SLAM3.so" CACHE FILEPATH
    "Pinned and patched ORB-SLAM3 shared library")
set(OVRS_PANGOLIN_PREFIX
    "${CMAKE_CURRENT_SOURCE_DIR}/.deps/install/pangolin-v0.6" CACHE PATH
    "Pinned Pangolin prefix used by the ORB-SLAM3 build")
set(OVRS_GLEW_PREFIX
    "${CMAKE_CURRENT_SOURCE_DIR}/.deps/install/glew-cmake" CACHE PATH
    "Pinned GLEW prefix used by the Pangolin build")
set(OVRS_PANGOLIN_LIBRARY
    "${OVRS_PANGOLIN_PREFIX}/lib/libpangolin.so" CACHE FILEPATH
    "Pinned Pangolin shared library used by ORB-SLAM3")
set(OVRS_GLEW_LIBRARY
    "${OVRS_GLEW_PREFIX}/lib/libglew.so" CACHE FILEPATH
    "Pinned GLEW shared library used by Pangolin")

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
if(OVRS_ENABLE_ORBSLAM3)
  if(NOT OVRS_ENABLE_HARDWARE)
    message(FATAL_ERROR
      "OVRS_ENABLE_ORBSLAM3 requires OVRS_ENABLE_HARDWARE for live capture")
  endif()
  list(APPEND _ovrs_opencv_components
    core imgproc features2d calib3d imgcodecs highgui)
endif()
if(_ovrs_opencv_components)
  list(REMOVE_DUPLICATES _ovrs_opencv_components)
  find_package(OpenCV 4 REQUIRED COMPONENTS ${_ovrs_opencv_components})
endif()

if(OVRS_ENABLE_ORBSLAM3)
  foreach(_ovrs_orb_required IN ITEMS
      "${OVRS_ORBSLAM3_SOURCE}/include/System.h"
      "${OVRS_ORBSLAM3_SOURCE}/Thirdparty/Sophus/sophus/se3.hpp"
      "${OVRS_PANGOLIN_PREFIX}/include/pangolin/pangolin.h"
      "${OVRS_GLEW_PREFIX}/include/GL/glew.h"
      "${OVRS_PANGOLIN_LIBRARY}"
      "${OVRS_GLEW_LIBRARY}"
      "${OVRS_ORBSLAM3_SOURCE}/Thirdparty/DBoW2/lib/libDBoW2.so"
      "${OVRS_ORBSLAM3_SOURCE}/Thirdparty/g2o/lib/libg2o.so"
      "${OVRS_ORBSLAM3_LIBRARY}")
    if(NOT EXISTS "${_ovrs_orb_required}")
      message(FATAL_ERROR
        "Pinned ORB-SLAM3 live dependency is incomplete: "
        "${_ovrs_orb_required}. See docs/orbslam3_live.md.")
    endif()
  endforeach()
  file(SHA256
    "${CMAKE_CURRENT_SOURCE_DIR}/patches/orbslam3-atlas-serialization-integrity.patch"
    _ovrs_orb_patch_sha256)
  if(NOT _ovrs_orb_patch_sha256 STREQUAL OVRS_ORBSLAM3_PATCH_SHA256)
    message(FATAL_ERROR
      "ORB-SLAM3 patch hash differs from the reviewed dependency pin")
  endif()
  execute_process(
    COMMAND git -C "${OVRS_ORBSLAM3_SOURCE}" rev-parse HEAD
    OUTPUT_VARIABLE _ovrs_orb_commit
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _ovrs_orb_git_status)
  if(NOT _ovrs_orb_git_status EQUAL 0 OR
     NOT _ovrs_orb_commit STREQUAL OVRS_ORBSLAM3_COMMIT)
    message(FATAL_ERROR
      "ORB-SLAM3 checkout is not at ${OVRS_ORBSLAM3_COMMIT}")
  endif()
  file(SHA256 "${OVRS_ORBSLAM3_LIBRARY}" _ovrs_orb_library_sha256)
  find_package(Eigen3 3.3 REQUIRED NO_MODULE)
  find_package(OpenGL REQUIRED)
  find_package(OpenSSL REQUIRED COMPONENTS Crypto)
  add_library(ovrs_orbslam3_backend SHARED IMPORTED)
  set_target_properties(ovrs_orbslam3_backend PROPERTIES
    IMPORTED_LOCATION "${OVRS_ORBSLAM3_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES
      "${OVRS_ORBSLAM3_SOURCE};${OVRS_ORBSLAM3_SOURCE}/include;${OVRS_ORBSLAM3_SOURCE}/include/CameraModels;${OVRS_ORBSLAM3_SOURCE}/Thirdparty/Sophus;${OVRS_PANGOLIN_PREFIX}/include;${OVRS_GLEW_PREFIX}/include")
endif()

add_library(ovrs_core
  src/app_support.cpp
  src/calibration_validation.cpp
  src/capture_mode.cpp
  src/config.cpp
  src/imu_startup_gate.cpp
  src/imu_synchronizer.cpp
  src/measurement_dispatcher.cpp
  src/orb_trajectory_gate.cpp
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

add_executable(ovrs_orbslam3_live apps/ovrs_orbslam3_live.cpp)
target_link_libraries(ovrs_orbslam3_live PRIVATE ovrs_core)
if(OVRS_ENABLE_HARDWARE)
  target_link_libraries(ovrs_orbslam3_live PRIVATE ovrs_realsense)
endif()
if(OVRS_ENABLE_ORBSLAM3)
  target_include_directories(ovrs_orbslam3_live SYSTEM PRIVATE
    ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(ovrs_orbslam3_live PRIVATE
    ovrs_orbslam3_backend Eigen3::Eigen ${OpenCV_LIBS}
    OpenGL::OpenGL OpenSSL::Crypto
    "${OVRS_PANGOLIN_LIBRARY}"
    "${OVRS_GLEW_LIBRARY}"
    "${OVRS_ORBSLAM3_SOURCE}/Thirdparty/DBoW2/lib/libDBoW2.so"
    "${OVRS_ORBSLAM3_SOURCE}/Thirdparty/g2o/lib/libg2o.so")
  target_compile_definitions(ovrs_orbslam3_live PRIVATE
    OVRS_HAS_ORBSLAM3=1
    OVRS_OPENCV_VERSION="${OpenCV_VERSION}"
    OVRS_ORBSLAM3_COMMIT_VALUE="${OVRS_ORBSLAM3_COMMIT}"
    OVRS_ORBSLAM3_PATCH_SHA256_VALUE="${OVRS_ORBSLAM3_PATCH_SHA256}"
    OVRS_ORBSLAM3_LIBRARY_SHA256_VALUE="${_ovrs_orb_library_sha256}"
    OVRS_ORBSLAM3_LIBRARY_PATH_VALUE="${OVRS_ORBSLAM3_LIBRARY}")
  set_property(TARGET ovrs_orbslam3_live APPEND PROPERTY BUILD_RPATH
    "${OVRS_ORBSLAM3_SOURCE}/lib"
    "${OVRS_ORBSLAM3_SOURCE}/Thirdparty/DBoW2/lib"
    "${OVRS_ORBSLAM3_SOURCE}/Thirdparty/g2o/lib")
endif()
ovrs_enable_project_warnings(ovrs_orbslam3_live)
ovrs_enable_project_sanitizers(ovrs_orbslam3_live)

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
    add_test(
      NAME ovrs_vislam_research_scripts_test
      COMMAND "${Python3_EXECUTABLE}"
              "${CMAKE_CURRENT_SOURCE_DIR}/tests/vislam_research_scripts_test.py")
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
                ovrs_orbslam3_live
        RUNTIME DESTINATION bin)
