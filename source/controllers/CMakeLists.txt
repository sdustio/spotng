list(APPEND srcs
  leg_ctrl.cc
  state_cmd.cc
  jpos_init.cc
)

list(TRANSFORM srcs PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")

set(SOURCES ${SOURCES} ${srcs} PARENT_SCOPE)
