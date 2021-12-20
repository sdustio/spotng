project(eiquadprog
  LANGUAGES CXX
  VERSION 1.2.3
)

# Project options
option(EIQUAD_TRACE_SOLVER "trace solver on stderr" OFF)

# Main Library
add_library(${PROJECT_NAME} STATIC
  repo/src/eiquadprog-fast.cpp
  repo/src/eiquadprog.cpp
)

if(EIQUAD_TRACE_SOLVER)
  target_compile_definitions(${PROJECT_NAME} PRIVATE TRACE_SOLVER)
endif(EIQUAD_TRACE_SOLVER)
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/repo/include>
)
if(BUILD_SHARED_LIBS)
  set_target_properties(${PROJECT_NAME}
    PROPERTIES
      POSITION_INDEPENDENT_CODE ON
)
endif(BUILD_SHARED_LIBS)
