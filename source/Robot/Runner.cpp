#include <chrono>

#include "sd/Robot/Runner.h"

namespace sd::robot
{
  using std::placeholders::_1;
  using namespace std::chrono_literals;

  Runner::Runner(std::shared_ptr<interface::Interface> itf) : Node(NODE_NAME, NODE_NS), mInterface(std::move(itf))
  {
    mDesiredStateCmd = std::make_shared<dynamics::DesiredStateCmd>();

    Init();

    mSPITimer = this->create_wall_timer(
        ctrldt::SPI, [this]() { this->mInterface->RunSPI(); });
    mIMUTimer = this->create_wall_timer(
        ctrldt::IMU, [this]() { this->mInterface->RunIMU(); });
    mDYNTimer = this->create_wall_timer(
        ctrldt::DYN, [this]() { this->Run(); });
  }

  bool Runner::Init()
  {
    mInterface->Init();

    mDriverCmdSub = this->create_subscription<sdrobot_api::msg::DriverCmd>(
        TOPIC_CMD, 10, [this](const sdrobot_api::msg::DriverCmd::SharedPtr msg) {this->handleDriverCmd(std::move(msg)); });

    return true;
  }

  bool Runner::Run()
  {
    return true;
  }

  void Runner::handleDriverCmd(const sdrobot_api::msg::DriverCmd::SharedPtr msg) const
  {
    mDesiredStateCmd->UpdateCmd(
        msg->mv[0], msg->mv[1], msg->tr,
        msg->pa, static_cast<sd::robot::dynamics::Mode>(msg->mode));
  }

} // namespace sd::robot
