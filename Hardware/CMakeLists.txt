FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_executable(HW ${headers} ${sources})
target_link_libraries(HW
  Controller
  Robot
)

install(TARGETS
  HW
  DESTINATION lib/${PROJECT_NAME})
