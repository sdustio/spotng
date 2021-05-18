#include "robot/runner.h"
#include "hardware/interface.h"

int main(int argc, char* argv[]){
  std::shared_ptr<sd::robot::Interface> itf = std::make_shared<sd::hardware::RobotInterface>();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sd::robot::Runner>(itf));
  rclcpp::shutdown();

  return 0;
}
