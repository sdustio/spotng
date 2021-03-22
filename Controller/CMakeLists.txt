find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library(Controller SHARED ${headers} ${sources})
ament_target_dependencies(Controller rclcpp std_msgs Eigen3)

install(TARGETS
  Controller
  DESTINATION lib/${PROJECT_NAME})
