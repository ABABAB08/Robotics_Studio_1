# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_lab1_image_subscribing_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED lab1_image_subscribing_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(lab1_image_subscribing_FOUND FALSE)
  elseif(NOT lab1_image_subscribing_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(lab1_image_subscribing_FOUND FALSE)
  endif()
  return()
endif()
set(_lab1_image_subscribing_CONFIG_INCLUDED TRUE)

# output package information
if(NOT lab1_image_subscribing_FIND_QUIETLY)
  message(STATUS "Found lab1_image_subscribing: 0.0.0 (${lab1_image_subscribing_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'lab1_image_subscribing' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${lab1_image_subscribing_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(lab1_image_subscribing_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${lab1_image_subscribing_DIR}/${_extra}")
endforeach()
