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
  wbc.cc
  wbc/wbic.cc
  wbc/kin_wbc.cc
  wbc/task/body_pos.cc
  wbc/task/body_ori.cc
  wbc/task/link_pos.cc
  wbc/contact/single.cc
)

list(TRANSFORM srcs PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")

set(SOURCES ${SOURCES} ${srcs} PARENT_SCOPE)
