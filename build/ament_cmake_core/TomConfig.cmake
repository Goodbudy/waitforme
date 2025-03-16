# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_Tom_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED Tom_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(Tom_FOUND FALSE)
  elseif(NOT Tom_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(Tom_FOUND FALSE)
  endif()
  return()
endif()
set(_Tom_CONFIG_INCLUDED TRUE)

# output package information
if(NOT Tom_FIND_QUIETLY)
  message(STATUS "Found Tom: 0.0.1 (${Tom_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'Tom' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${Tom_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(Tom_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${Tom_DIR}/${_extra}")
endforeach()
