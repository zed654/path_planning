# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_hybrid_a_star_algorithm_pkg_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED hybrid_a_star_algorithm_pkg_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(hybrid_a_star_algorithm_pkg_FOUND FALSE)
  elseif(NOT hybrid_a_star_algorithm_pkg_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(hybrid_a_star_algorithm_pkg_FOUND FALSE)
  endif()
  return()
endif()
set(_hybrid_a_star_algorithm_pkg_CONFIG_INCLUDED TRUE)

# output package information
if(NOT hybrid_a_star_algorithm_pkg_FIND_QUIETLY)
  message(STATUS "Found hybrid_a_star_algorithm_pkg: 0.4.0 (${hybrid_a_star_algorithm_pkg_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'hybrid_a_star_algorithm_pkg' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message(WARNING "${_msg}")
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(hybrid_a_star_algorithm_pkg_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${hybrid_a_star_algorithm_pkg_DIR}/${_extra}")
endforeach()
