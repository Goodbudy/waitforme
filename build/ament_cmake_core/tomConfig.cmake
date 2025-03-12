# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_tom_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED tom_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(tom_FOUND FALSE)
  elseif(NOT tom_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(tom_FOUND FALSE)
  endif()
  return()
endif()
set(_tom_CONFIG_INCLUDED TRUE)

# output package information
if(NOT tom_FIND_QUIETLY)
  message(STATUS "Found tom: 0.0.1 (${tom_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'tom' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${tom_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(tom_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${tom_DIR}/${_extra}")
endforeach()
