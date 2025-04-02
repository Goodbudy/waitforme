# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_hallie_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED hallie_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(hallie_FOUND FALSE)
  elseif(NOT hallie_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(hallie_FOUND FALSE)
  endif()
  return()
endif()
set(_hallie_CONFIG_INCLUDED TRUE)

# output package information
if(NOT hallie_FIND_QUIETLY)
  message(STATUS "Found hallie: 1.0.0 (${hallie_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'hallie' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${hallie_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(hallie_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${hallie_DIR}/${_extra}")
endforeach()
