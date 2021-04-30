#include <memory>

#include "sd/Robot/Runner.h"
#include "Robot/Interface/Hardware/Interface.hpp"

int main(int argc, char* argv[]){
  using namespace sd::robot;

  std::shared_ptr<interface::Interface> itf = std::make_shared<interface::hardware::Interface>();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Runner>(itf));
  rclcpp::shutdown();

  return 0;
}
