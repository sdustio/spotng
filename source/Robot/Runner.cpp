#include "Robot/Runner.hpp"

namespace sd::robot
{
  using std::placeholders::_1;
  using namespace std::chrono_literals;

  Runner::Runner(std::shared_ptr<interface::Interface> itf) :
    Node(ros::NODE_NAME, ros::NODE_NS),
    mInterface(std::move(itf)),
    mDesiredStateCmd(ctrldt::DYNsec)
  {
    Init();
  }

  bool Runner::Init()
  {
    mInterface->Init();
    mSPITimer = this->create_wall_timer(
        ctrldt::SPI, [this]() { this->mInterface->RunSPI(); });
    mIMUTimer = this->create_wall_timer(
        ctrldt::IMU, [this]() { this->mInterface->RunIMU(); });


    mDriverCmdSub = this->create_subscription<sdrobot_api::msg::DriverCmd>(
        ros::TOPIC_CMD, 10, [this](const sdrobot_api::msg::DriverCmd::SharedPtr msg) {this->handleDriverCmd(std::move(msg)); });

    mDYNTimer = this->create_wall_timer(
        ctrldt::DYN, [this]() { this->Run(); });


    return true;
  }

  bool Runner::Run()
  {
    return true;
  }

  void Runner::handleDriverCmd(const sdrobot_api::msg::DriverCmd::SharedPtr msg)
  {
    mDesiredStateCmd.UpdateCmd(
        msg->mv[0], msg->mv[1], msg->tr,
        msg->pa, static_cast<dynamics::Mode>(msg->mode));
  }

} // namespace sd::robot
