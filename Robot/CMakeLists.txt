FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library(Robot SHARED ${headers} ${sources})

install(TARGETS
  Robot
  DESTINATION lib/${PROJECT_NAME})
