cmake_minimum_required(VERSION 3.15)

project(sdrobot
  LANGUAGES CXX
  VERSION 1.0.0
)

### Project properties
set(PACKAGE_ORG sd)
set(PACKAGE_DESCRIPTION "sdrobot lib")
set(TARGET_NAME ${PROJECT_NAME}Targets)

### CXX
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
message(STATUS "Project will be installed to ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Build type set to ${CMAKE_BUILD_TYPE}")


#### Library
find_package(Eigen3 REQUIRED)
find_package(qpOASES REQUIRED)
find_package(eiquadprog REQUIRED)

add_library(${PROJECT_NAME} SHARED ${sources})
target_sources(${PROJECT_NAME} PRIVATE
  source/geometry_impl.cc
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen qpOASES::qpOASES eiquadprog::eiquadprog
)

include(GenerateExportHeader)
generate_export_header(${PROJECT_NAME}
  EXPORT_FILE_NAME "${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}/export.h"
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/source>
)
set_target_properties(${PROJECT_NAME}
  PROPERTIES
    POSITION_INDEPENDENT_CODE 1
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN 1
    SOVERSION "${PROJECT_VERSION_MAJOR}"
)

if(WIN32 AND NOT CYGWIN)
  set(INSTALL_CMAKEDIR CMake)
else()
  set(INSTALL_CMAKEDIR share/cmake/${PROJECT_NAME})
endif()

install(
  TARGETS
    ${PROJECT_NAME}
  EXPORT ${TARGET_NAME}
  LIBRARY
    COMPONENT lib
  ARCHIVE
    COMPONENT lib
)

install(
  DIRECTORY
    "${CMAKE_SOURCE_DIR}/include/"
    "${CMAKE_BINARY_DIR}/include/"
  TYPE INCLUDE
  FILES_MATCHING
    PATTERN "*.h"
)

install(
  EXPORT
    ${TARGET_NAME}
  NAMESPACE
    "${PROJECT_NAME}::"
  DESTINATION
    ${INSTALL_CMAKEDIR}
  COMPONENT
    dev
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
      COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  ${CMAKE_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${INSTALL_CMAKEDIR}
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION
    ${INSTALL_CMAKEDIR}
)
