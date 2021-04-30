#include "sd/Robot/Runner.h"

namespace sd::robot
{
  using std::placeholders::_1;

  Runner::Runner(std::shared_ptr<interface::Interface> itf): Node(NODE_NAME, NODE_NS)
  {
    mInterface = std::move(itf);
    mDesiredStateCmd = std::make_shared<dynamics::DesiredStateCmd>();

    Init();
  }

  bool Runner::Init()
  {
    mInterface->Init();

    mDriverCmdSub = this->create_subscription<sdrobot_api::msg::DriverCmd>(
      TOPIC_CMD, 10, std::bind(&Runner::handleDriverCmd, this, _1)
    );

    return true;
  }

  bool Runner::Run()
  {
    printf("Run");
    return true;
  }

  void Runner::handleDriverCmd(const sdrobot_api::msg::DriverCmd::SharedPtr msg) const
  {
    mDesiredStateCmd->UpdateCmd(
      msg->mv[0], msg->mv[1], msg->tr,
      msg->pa, static_cast<sd::robot::dynamics::Mode>(msg->mode));
  }

  int Run(std::shared_ptr<interface::Interface> itf, int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Runner>(std::move(itf)));
    rclcpp::shutdown();
    return 0;
  }
} // namespace sd::robot
