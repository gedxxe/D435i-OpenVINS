function(ovrs_compute_source_fingerprint source_dir output_variable)
  if(NOT IS_DIRECTORY "${source_dir}")
    message(FATAL_ERROR "Fingerprint source directory does not exist")
  endif()

  file(GLOB_RECURSE _ovrs_fingerprint_sources
    "${source_dir}/apps/*.cpp"
    "${source_dir}/include/ovrs/*.hpp"
    "${source_dir}/src/*.cpp")
  file(GLOB _ovrs_fingerprint_cmake "${source_dir}/cmake/*.cmake")
  file(GLOB _ovrs_fingerprint_patches "${source_dir}/patches/*.patch")
  list(APPEND _ovrs_fingerprint_sources
    "${source_dir}/CMakeLists.txt"
    "${source_dir}/CMakePresets.json"
    "${source_dir}/VERSION"
    ${_ovrs_fingerprint_cmake}
    ${_ovrs_fingerprint_patches})
  list(REMOVE_DUPLICATES _ovrs_fingerprint_sources)
  list(SORT _ovrs_fingerprint_sources)

  set(_ovrs_fingerprint_input "")
  foreach(_ovrs_source IN LISTS _ovrs_fingerprint_sources)
    if(NOT EXISTS "${_ovrs_source}" OR IS_DIRECTORY "${_ovrs_source}")
      message(FATAL_ERROR
        "Fingerprint input is missing: ${_ovrs_source}")
    endif()
    file(SHA256 "${_ovrs_source}" _ovrs_file_hash)
    file(RELATIVE_PATH _ovrs_relative_source
      "${source_dir}" "${_ovrs_source}")
    string(APPEND _ovrs_fingerprint_input
      "${_ovrs_relative_source}:${_ovrs_file_hash}\n")
  endforeach()

  string(SHA256 _ovrs_source_fingerprint "${_ovrs_fingerprint_input}")
  set("${output_variable}" "${_ovrs_source_fingerprint}" PARENT_SCOPE)
endfunction()

if(CMAKE_SCRIPT_MODE_FILE)
  if(NOT DEFINED OVRS_SOURCE_DIR)
    message(FATAL_ERROR "OVRS_SOURCE_DIR is required in script mode")
  endif()
  ovrs_compute_source_fingerprint(
    "${OVRS_SOURCE_DIR}" _ovrs_script_fingerprint)
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E echo "${_ovrs_script_fingerprint}"
    COMMAND_ERROR_IS_FATAL ANY)
endif()
