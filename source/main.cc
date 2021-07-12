#include "robot/runner.h"
#include "hardware/interface.h"

int main(int argc, char* argv[]){
  sdrobot::robot::InterfacePtr itf = std::make_shared<sdrobot::hardware::RobotInterface>();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sdrobot::robot::Runner>(itf));
  rclcpp::shutdown();

  return 0;
}
