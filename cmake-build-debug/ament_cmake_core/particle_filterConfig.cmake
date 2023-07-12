# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_particle_filter_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED particle_filter_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(particle_filter_FOUND FALSE)
  elseif(NOT particle_filter_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(particle_filter_FOUND FALSE)
  endif()
  return()
endif()
set(_particle_filter_CONFIG_INCLUDED TRUE)

# output package information
if(NOT particle_filter_FIND_QUIETLY)
  message(STATUS "Found particle_filter: 0.0.0 (${particle_filter_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'particle_filter' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${particle_filter_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(particle_filter_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${particle_filter_DIR}/${_extra}")
endforeach()
