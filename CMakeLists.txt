cmake_minimum_required(VERSION 3.15)

project(sdquadx
  LANGUAGES CXX
  VERSION 1.0.0
)

### Project properties
set(PACKAGE_ORG sd)
set(PACKAGE_DESCRIPTION "sdquadx lib")
set(TARGET_NAME ${PROJECT_NAME}Targets)

### C
set(CMAKE_C_STANDARD 11)
### CXX
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  add_compile_options(/source-charset:utf-8)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
message(STATUS "Project will be installed to ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Build type set to ${CMAKE_BUILD_TYPE}")

option(BUILD_TESTING "Build examples.")

#### Library
find_package(Eigen3 REQUIRED)
find_package(qpOASES REQUIRED)
find_package(eiquadprog REQUIRED)
find_package(spdlog REQUIRED)

set(LIB_SOURCES
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

add_library(${PROJECT_NAME} SHARED)
target_sources(${PROJECT_NAME} PRIVATE ${LIB_SOURCES})
target_link_libraries(${PROJECT_NAME}
  PRIVATE Eigen3::Eigen qpOASES::qpOASES eiquadprog::eiquadprog spdlog::spdlog
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

target_compile_definitions(${PROJECT_NAME} PRIVATE
  $<$<STREQUAL:$<UPPER_CASE:${CMAKE_BUILD_TYPE}>,DEBUG>:DEBUG_MODE>
)

set_target_properties(${PROJECT_NAME}
  PROPERTIES
    POSITION_INDEPENDENT_CODE 1
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN 1
    SOVERSION "${PROJECT_VERSION_MAJOR}"
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

if(BUILD_TESTING)
  add_executable(testmain)
  target_sources(testmain PRIVATE
      test/main.cc
      test/example.cc
      test/itf/impl.cc
  )
  target_include_directories(testmain PRIVATE
    ${CMAKE_BINARY_DIR}/include
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/test
  )
  target_link_libraries(testmain
    PRIVATE ${PROJECT_NAME}
  )
endif()
