find_package(rclcpp REQUIRED)
find_package(sdrobot_interfaces REQUIRED)
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

add_executable(runner
  ${SOURCES}
)
target_include_directories(runner
  PUBLIC ${CMAKE_SOURCE_DIR}/include
  PRIVATE ${CMAKE_SOURCE_DIR}/vendor
)
target_link_libraries(runner
  quadprog
)
ament_target_dependencies(runner
  rclcpp sdrobot_interfaces sensor_msgs Eigen3
)

install(TARGETS
  runner
  DESTINATION lib/${PROJECT_NAME})
