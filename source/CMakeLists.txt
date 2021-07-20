find_package(rclcpp REQUIRED)
find_package(sdrobot_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(qpOASES REQUIRED)
find_package(eiquadprog REQUIRED)

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
)
target_link_libraries(runner
  Eigen3::Eigen qpOASES::qpOASES eiquadprog::eiquadprog
)

ament_target_dependencies(runner
  rclcpp sdrobot_interfaces sensor_msgs
)

install(TARGETS
  runner
  DESTINATION lib/${PROJECT_NAME})
