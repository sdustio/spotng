#include "robot/runner.h"

#include "dynamics/spatial.h"

namespace sd::robot
{
  using std::placeholders::_1;
  using namespace std::chrono_literals;

  Runner::Runner(std::shared_ptr<Interface> itf) :
    Node(ros::kNodeName, ros::kNodeNs),
    interface_(std::move(itf)),
    desired_state_cmd_(ctrldt::DYNsec)
  {
    Init();
  }

  bool Runner::Init()
  {
    interface_->Init();
    spi_timer_ = this->create_wall_timer(
        ctrldt::kSPI, [this]() { this->interface_->RunSPI(); });
    imu_timer_ = this->create_wall_timer(
        ctrldt::kIMU, [this]() { this->interface_->RunIMU(); });


    driver_cmd_sub_ = this->create_subscription<sdrobot_api::msg::DriverCmd>(
        ros::kTopicCmd, 10, [this](const sdrobot_api::msg::DriverCmd::SharedPtr msg) {this->HandleDriverCmd(std::move(msg)); });

    dyn_timer_ = this->create_wall_timer(
        ctrldt::kDYN, [this]() { this->Run(); });

    return true;
  }

  bool Runner::Run()
  {
    return true;
  }

  void Runner::HandleDriverCmd(const sdrobot_api::msg::DriverCmd::SharedPtr msg)
  {
    desired_state_cmd_.UpdateCmd(
        msg->mv[0], msg->mv[1], msg->tr,
        msg->pa, static_cast<dynamics::Mode>(msg->mode));
  }

} // namespace sd::robot
