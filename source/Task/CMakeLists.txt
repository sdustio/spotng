list(APPEND srcs
  PeriodicTask.cpp
)

list(TRANSFORM srcs PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")

set(LIB_SOURCES ${LIB_SOURCES} ${srcs} PARENT_SCOPE)
