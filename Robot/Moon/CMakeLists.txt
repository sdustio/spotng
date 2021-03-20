FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library(MoonRobot SHARED ${headers} ${sources})

install(TARGETS
  MoonRobot
  DESTINATION lib/${PROJECT_NAME})
