FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_executable(MoonSim ${headers} ${sources})
target_link_libraries(MoonSim
  MoonController
  MoonRobot
)

install(TARGETS
  MoonSim
  DESTINATION lib/${PROJECT_NAME})
