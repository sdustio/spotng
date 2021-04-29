add_executable(RobotExe
  main.cpp
)

target_link_libraries(RobotExe RobotLib)

install(TARGETS
  RobotExe
  DESTINATION bin/${PROJECT_NAME})
