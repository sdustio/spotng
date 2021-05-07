find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sdrobot_api REQUIRED)
find_package(Eigen3 REQUIRED)

set(LIB_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/Lib.cpp
)

add_subdirectory(Robot)

add_library(RobotLib STATIC
  ${LIB_SOURCES}
)
ament_target_dependencies(RobotLib sdrobot_api rclcpp std_msgs Eigen3)

set(EXE_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/Exe/main.cpp
)

add_subdirectory(Exe)

add_executable(RobotExe
  ${EXE_SOURCES}
)
target_link_libraries(RobotExe RobotLib)

install(TARGETS
  RobotExe RobotLib
  DESTINATION lib/${PROJECT_NAME})
