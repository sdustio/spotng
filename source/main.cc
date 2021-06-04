#include "robot/runner.h"
#include "hardware/interface.h"

int main(int argc, char* argv[]){
  sd::robot::InterfacePtr itf = std::make_unique<sd::hardware::RobotInterface>();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sd::robot::Runner>(std::move(itf)));
  rclcpp::shutdown();

  return 0;
}
