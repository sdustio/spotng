find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(Controller STATIC
  src/Interface.cpp
)
target_include_directories(Controller PUBLIC include)
ament_target_dependencies(Controller rclcpp std_msgs Eigen3)

install(TARGETS
  Controller
  DESTINATION lib/${PROJECT_NAME})
