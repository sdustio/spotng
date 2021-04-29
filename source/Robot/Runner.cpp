#include <memory>

#include "sd/Robot/Runner.h"

namespace sd::robot
{
  using std::placeholders::_1;

  Runner::Runner(interface::Interface* itf): Node(NODE_NAME, NODE_NS)
  {
    mInterface = itf;
  }

  int Runner::Init()
  {
    mInterface->Init();

    mDesiredStateCmd = new dynamics::DesiredStateCmd();

    mDriverCmdSub = this->create_subscription<sdrobot_api::msg::DriverCmd>(
      TOPIC_CMD, 10, std::bind(&Runner::handleDriverCmd, this, _1)
    );
  }

  void Runner::Run()
  {
    Init();
  }

  void Runner::handleDriverCmd(const sdrobot_api::msg::DriverCmd::SharedPtr msg) const
  {
    mDesiredStateCmd->UpdateCmd(
      msg->mv[0], msg->mv[1], msg->tr,
      msg->pa, static_cast<sd::robot::dynamics::Mode>(msg->mode));
  }
} // namespace sd::robot
