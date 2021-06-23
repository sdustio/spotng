list(APPEND srcs
  leg.cc
  state_cmd.cc
  jpos_init.cc
  gait_skd.cc
  fsm.cc
  fsm/safety_checker.cc
  fsm/state/off.cc
  fsm/state/ready.cc
  fsm/state/stand_up.cc
  fsm/state/locomotion.cc
  fsm/state/balance_stand.cc
  fsm/state/recovery_stand.cc
)

list(TRANSFORM srcs PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")

set(SOURCES ${SOURCES} ${srcs} PARENT_SCOPE)
