add_executable(sdrobot
  src/main.cpp
  src/Bridge.cpp
)
target_include_directories(sdrobot PRIVATE include)
target_link_libraries(sdrobot PRIVATE
  Controller
  Robot
)

install(TARGETS
  sdrobot
  DESTINATION lib/${PROJECT_NAME})
