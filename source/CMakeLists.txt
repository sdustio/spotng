find_package(rclcpp REQUIRED)
find_package(sdrobot_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/main.cc
)

add_subdirectory(dynamics)
add_subdirectory(controllers)
add_subdirectory(estimators)
add_subdirectory(robot)
add_subdirectory(hardware)

add_executable(sdrobot
  ${SOURCES}
)
ament_target_dependencies(sdrobot rclcpp sdrobot_msgs sensor_msgs Eigen3)

install(TARGETS
  sdrobot
  DESTINATION lib/${PROJECT_NAME})
