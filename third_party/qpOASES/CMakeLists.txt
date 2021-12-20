cmake_minimum_required(VERSION 3.15)
project(qpOASES
  LANGUAGES CXX
  VERSION 3.2.1
  )

option(QPOASES_AVOID_LA_NAMING_CONFLICTS "If ON, avoid to re-defined symbols that conflict with Blas/Lapack provided functions." OFF)

############################################################
#################### build and install #####################
############################################################

# compile qpOASES libraries
file(GLOB SRC repo/src/*.cpp)

# library
add_library(qpOASES STATIC ${SRC})
target_include_directories(qpOASES PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/repo/include>
)
target_compile_definitions(qpOASES PRIVATE
  __NO_COPYRIGHT__
  $<$<BOOL:${QPOASES_AVOID_LA_NAMING_CONFLICTS}>:__AVOID_LA_NAMING_CONFLICTS__>
  $<$<BOOL:${UNIX}>:LINUX>
  $<$<BOOL:${WIN32}>:WIN32>
)

if(UNIX)
  target_compile_options(qpOASES PRIVATE
    -O3 -finline-functions
  )
elseif(WIN32)
  target_compile_options(qpOASES PRIVATE
    -nologo -EHsc
  )
endif(UNIX)

if(BUILD_SHARED_LIBS)
  set_target_properties(${PROJECT_NAME}
    PROPERTIES
      POSITION_INDEPENDENT_CODE ON
  )
endif(BUILD_SHARED_LIBS)
