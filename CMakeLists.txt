cmake_minimum_required(VERSION 3.15)

project(spotng
  LANGUAGES CXX
  VERSION 1.0.1
)

### Project properties
set(PACKAGE_ORG sd)
set(PACKAGE_DESCRIPTION "Core dynamics engine for OpenSpot, a quadruped droid developed by sdustio")
set(TARGET_NAME ${PROJECT_NAME}Targets)

### C
set(CMAKE_C_STANDARD 11)
### CXX
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif(NOT CMAKE_BUILD_TYPE)
message(STATUS "Project will be installed to ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Build type set to ${CMAKE_BUILD_TYPE}")

option(BUILD_SHARED_LIBS "Build shared libs." ON)
option(SPOTNG_BUILD_TESTS "Build tests." OFF)
if(SPOTNG_BUILD_TESTS)
  set(BUILD_SHARED_LIBS OFF)
  message(STATUS "force building static libs when build tests")
endif(SPOTNG_BUILD_TESTS)


#### Library
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)

add_subdirectory(third_party)

add_library(${PROJECT_NAME})
target_sources(${PROJECT_NAME} PRIVATE
  source/utils/eigen.cc
  source/utils/debug.cc
  source/math/algebra.cc
  source/math/interpolate.cc
  source/dynamics/rotation.cc
  source/dynamics/inertia.cc
  source/model/float_base.cc
  source/model/quadruped_impl.cc
  source/drive/drive_ctrl_impl.cc
  source/estimate/impl.cc
  source/estimate/joints.cc
  source/estimate/orientation.cc
  source/estimate/pos_vel.cc
  source/estimate/contact.cc
  source/skd/od_gait.cc
  source/skd/state_des.cc
  source/skd/foot_swing.cc
  source/mpc/qp.cc
  source/mpc/cmpc.cc
  source/wbc/task/body_ori.cc
  source/wbc/task/body_pos.cc
  source/wbc/task/foot_pos.cc
  source/wbc/task/foot_contact.cc
  source/wbc/wbic.cc
  source/fsm/impl.cc
  source/fsm/legctrl.cc
  source/fsm/state/init.cc
  source/fsm/state/balance_stand.cc
  source/fsm/state/recovery_stand.cc
  source/fsm/state/locomotion.cc
  source/robot/robot_impl.cc
  source/robot/options_impl.cc
)
target_link_libraries(${PROJECT_NAME}
  PRIVATE Eigen3::Eigen spdlog::spdlog_header_only qpOASES eiquadprog
)

include(GenerateExportHeader)
generate_export_header(${PROJECT_NAME}
  EXPORT_FILE_NAME "${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}/export.h"
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/source>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  $<$<STREQUAL:$<UPPER_CASE:${CMAKE_BUILD_TYPE}>,DEBUG>:DEBUG_MODE>
  $<$<CONFIG:Debug>:DEBUG_MODE>
)

set_target_properties(${PROJECT_NAME}
  PROPERTIES
    VERSION ${PROJECT_VERSION}
)


if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  target_compile_options(${PROJECT_NAME} PRIVATE
    /source-charset:utf-8
  )
endif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall -Wextra -Wpedantic
  )
endif(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")

if(BUILD_SHARED_LIBS)
set_target_properties(${PROJECT_NAME}
  PROPERTIES
    SOVERSION ${PROJECT_VERSION_MAJOR}
    CXX_VISIBILITY_PRESET hidden
)

set(INSTALL_CMAKEDIR share/cmake/${PROJECT_NAME})

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
  COMPONENT
    dev
  TYPE INCLUDE
  FILES_MATCHING
    PATTERN "*.h"
)

install(
  EXPORT
    ${TARGET_NAME}
  COMPONENT
    dev
  NAMESPACE
    "${PROJECT_NAME}::"
  DESTINATION
    ${INSTALL_CMAKEDIR}
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
  COMPONENT
    dev
  DESTINATION
    ${INSTALL_CMAKEDIR}
)
endif(BUILD_SHARED_LIBS)

if(SPOTNG_BUILD_TESTS)
  find_program(MEMORYCHECK_COMMAND NAMES valgrind)
  set(MEMORYCHECK_COMMAND_OPTIONS "--trace-children=yes --leak-check=full")
  include (CTest)

  enable_testing()

  function(spotng_test test_file)
    get_filename_component(test_target_name ${test_file} NAME_WE)

    add_executable(${test_target_name})
    target_sources(${test_target_name}
      PRIVATE
        test/testitf.cc
        ${test_file}
    )
    target_link_libraries(${test_target_name} PRIVATE ${PROJECT_NAME} qpOASES eiquadprog)
    target_link_libraries(${test_target_name} PRIVATE Eigen3::Eigen spdlog::spdlog)
    target_include_directories(${test_target_name}
      PRIVATE
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/test>
    )
    add_test(NAME ${test_target_name} COMMAND ${test_target_name})
  endfunction(spotng_test)

  set(TEST_FILES
    test/example1_test.cc
    test/dynamics_rotation_test.cc
  )
  foreach(ELEMENT ${TEST_FILES})
    spotng_test(${ELEMENT})
  endforeach(ELEMENT ${TEST_FILES})

endif(SPOTNG_BUILD_TESTS)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(packing)
