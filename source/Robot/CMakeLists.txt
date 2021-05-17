list(APPEND srcs
  Model.cpp
  Runner.cpp
)

list(TRANSFORM srcs PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")

set(SOURCES ${SOURCES} ${srcs} PARENT_SCOPE)
