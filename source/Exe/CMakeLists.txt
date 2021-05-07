list(APPEND srcs
  Hardware/Interface.cpp
)

list(TRANSFORM srcs PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")

set(EXE_SOURCES ${EXE_SOURCES} ${srcs} PARENT_SCOPE)
