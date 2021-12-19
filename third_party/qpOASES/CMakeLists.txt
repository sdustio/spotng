cmake_minimum_required(VERSION 3.15)
project(qpOASES
  LANGUAGES CXX
  VERSION 3.2.1
  )

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(QPOASES_AVOID_LA_NAMING_CONFLICTS "If ON, avoid to re-defined symbols that conflict with Blas/Lapack provided functions." OFF)


############################################################
#################### compiler flags ########################
############################################################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__NO_COPYRIGHT__")
if(QPOASES_AVOID_LA_NAMING_CONFLICTS)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__AVOID_LA_NAMING_CONFLICTS__")
endif()

if(UNIX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wfloat-equal -Wshadow -DLINUX")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_DEBUG} -O3 -finline-functions")
elseif(WINDOWS)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -nologo -EHsc -DWIN32")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D__DEBUG__")

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
