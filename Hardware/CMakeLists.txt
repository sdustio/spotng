FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_executable(MoonHW ${headers} ${sources})
target_link_libraries(MoonHW
  MoonController
  MoonRobot
)

install(TARGETS
  MoonHW
  DESTINATION lib/${PROJECT_NAME})
