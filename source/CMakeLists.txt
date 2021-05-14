find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sdrobot_api REQUIRED)
find_package(Eigen3 REQUIRED)

set(SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

add_subdirectory(Dynamics)
add_subdirectory(Robot)
add_subdirectory(Hardware)

add_executable(SdRobot
  ${SOURCES}
)
ament_target_dependencies(SdRobot sdrobot_api rclcpp std_msgs Eigen3)

install(TARGETS
  SdRobot
  DESTINATION lib/${PROJECT_NAME})
