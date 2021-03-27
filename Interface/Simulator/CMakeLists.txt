add_executable(sdrobot-sim
  src/main.cpp
)
target_include_directories(sdrobot-sim PRIVATE include)
target_link_libraries(sdrobot-sim PRIVATE
  Controller
  Robot
)

install(TARGETS
  sdrobot-sim
  DESTINATION lib/${PROJECT_NAME})
