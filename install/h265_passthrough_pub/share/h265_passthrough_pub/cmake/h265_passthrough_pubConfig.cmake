# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_h265_passthrough_pub_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED h265_passthrough_pub_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(h265_passthrough_pub_FOUND FALSE)
  elseif(NOT h265_passthrough_pub_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(h265_passthrough_pub_FOUND FALSE)
  endif()
  return()
endif()
set(_h265_passthrough_pub_CONFIG_INCLUDED TRUE)

# output package information
if(NOT h265_passthrough_pub_FIND_QUIETLY)
  message(STATUS "Found h265_passthrough_pub: 0.0.1 (${h265_passthrough_pub_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'h265_passthrough_pub' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${h265_passthrough_pub_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(h265_passthrough_pub_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${h265_passthrough_pub_DIR}/${_extra}")
endforeach()
