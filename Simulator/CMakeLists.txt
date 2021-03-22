FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_executable(Sim ${headers} ${sources})
target_link_libraries(Sim
  Controller
  Robot
)

install(TARGETS
  Sim
  DESTINATION lib/${PROJECT_NAME})
