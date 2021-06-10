list(APPEND srcs
  contact.cc
  orientation.cc
  pos_vel.cc
)

list(TRANSFORM srcs PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")

set(SOURCES ${SOURCES} ${srcs} PARENT_SCOPE)
