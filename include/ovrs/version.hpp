#pragma once

#ifndef OVRS_OPENCV_VERSION
#define OVRS_OPENCV_VERSION "not linked in this build"
#endif
#ifndef OVRS_CERES_VERSION
#define OVRS_CERES_VERSION "not linked in this build"
#endif
#ifndef OVRS_REALSENSE_VERSION
#define OVRS_REALSENSE_VERSION "not linked in this build"
#endif
#ifndef OVRS_PROJECT_VERSION
#define OVRS_PROJECT_VERSION "non-CMake audit build"
#endif
#ifndef OVRS_SOURCE_FINGERPRINT
#define OVRS_SOURCE_FINGERPRINT "non-CMake-audit"
#endif
#ifndef OVRS_OPENVINS_TAG_VALUE
#define OVRS_OPENVINS_TAG_VALUE "not linked in this build"
#endif
#ifndef OVRS_OPENVINS_COMMIT_VALUE
#define OVRS_OPENVINS_COMMIT_VALUE "not linked in this build"
#endif

namespace ovrs {
inline constexpr const char *project_version = OVRS_PROJECT_VERSION;
inline constexpr const char *source_fingerprint = OVRS_SOURCE_FINGERPRINT;
inline constexpr const char *openvins_tag = OVRS_OPENVINS_TAG_VALUE;
inline constexpr const char *openvins_commit = OVRS_OPENVINS_COMMIT_VALUE;
inline constexpr const char *ceres_version = OVRS_CERES_VERSION;
inline constexpr const char *opencv_version = OVRS_OPENCV_VERSION;
inline constexpr const char *realsense_version = OVRS_REALSENSE_VERSION;
#ifdef _MSC_VER
inline constexpr const char *compiler_version = "Microsoft Visual C++";
#else
inline constexpr const char *compiler_version = __VERSION__;
#endif
#ifdef NDEBUG
inline constexpr const char *build_type = "Release";
#else
inline constexpr const char *build_type = "Debug";
#endif
} // namespace ovrs
