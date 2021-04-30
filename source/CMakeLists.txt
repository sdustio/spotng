find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sdrobot_api REQUIRED)
find_package(Eigen3 REQUIRED)

set(LIB_SOURCES
  Lib.cpp
)

add_subdirectory(Task)
add_subdirectory(Robot)

add_library(RobotLib STATIC
  ${LIB_SOURCES}
)
ament_target_dependencies(RobotLib sdrobot_api rclcpp std_msgs Eigen3)

add_executable(RobotExe
  Exe/main.cpp
)
target_link_libraries(RobotExe RobotLib)

install(TARGETS
  RobotExe RobotLib
  DESTINATION lib/${PROJECT_NAME})
