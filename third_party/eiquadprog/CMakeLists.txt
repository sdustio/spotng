project(eiquadprog
  LANGUAGES CXX
  VERSION 1.2.3
)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Project options
option(EIQUAD_TRACE_SOLVER "trace solver on stderr" OFF)

# Project configuration
set(CXX_DISABLE_WERROR TRUE)

# Main Library
add_library(${PROJECT_NAME} STATIC
  repo/src/eiquadprog-fast.cpp
  repo/src/eiquadprog.cpp
)

if(EIQUAD_TRACE_SOLVER)
  target_compile_definitions(${PROJECT_NAME} PRIVATE TRACE_SOLVER)
endif()
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/repo/include>
)
