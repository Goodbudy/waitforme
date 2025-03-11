# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_andy_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED andy_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(andy_FOUND FALSE)
  elseif(NOT andy_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(andy_FOUND FALSE)
  endif()
  return()
endif()
set(_andy_CONFIG_INCLUDED TRUE)

# output package information
if(NOT andy_FIND_QUIETLY)
  message(STATUS "Found andy: 0.0.0 (${andy_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'andy' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${andy_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(andy_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${andy_DIR}/${_extra}")
endforeach()
