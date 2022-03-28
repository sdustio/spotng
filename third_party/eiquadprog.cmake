# Project options
option(EIQUAD_TRACE_SOLVER "trace solver on stderr" OFF)

# Main Library
add_library(eiquadprog STATIC
  eiquadprog/src/eiquadprog-fast.cpp
  eiquadprog/src/eiquadprog.cpp
)

if(EIQUAD_TRACE_SOLVER)
  target_compile_definitions(eiquadprog PRIVATE TRACE_SOLVER)
endif(EIQUAD_TRACE_SOLVER)
target_link_libraries(eiquadprog PRIVATE Eigen3::Eigen)
target_include_directories(eiquadprog PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/eiquadprog/include>
)
if(BUILD_SHARED_LIBS)
  set_target_properties(eiquadprog
    PROPERTIES
      POSITION_INDEPENDENT_CODE ON
)
endif(BUILD_SHARED_LIBS)
