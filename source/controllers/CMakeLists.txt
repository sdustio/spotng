list(APPEND srcs
  leg.cc
  state_cmd.cc
  jpos_init.cc
  fsm.cc
  fsm/state_ctrl.cc
  fsm/safety_checker.cc
  fsm/state/init.cc
  fsm/state/locomotion.cc
  fsm/state/balance_stand.cc
  fsm/state/recovery_stand.cc
)

list(TRANSFORM srcs PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")

set(SOURCES ${SOURCES} ${srcs} PARENT_SCOPE)
