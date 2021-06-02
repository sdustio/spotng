#include "robot/runner.h"

#include "dynamics/spatial.h"

namespace sd::robot
{
  using std::placeholders::_1;
  using namespace std::chrono_literals;

  Runner::Runner(std::shared_ptr<Interface> itf) :
    Node(ros::kNodeName, ros::kNodeNs),
    interface_(std::move(itf)),
    state_cmd_(ctrldt::DYNsec)
  {
    Init();
  }

  bool Runner::Init()
  {
    // interface init & run periodically
    interface_->Init();
    spi_timer_ = this->create_wall_timer(
        ctrldt::kSPI, [this]() { this->interface_->RunSPI(); });
    imu_timer_ = this->create_wall_timer(
        ctrldt::kIMU, [this]() { this->interface_->RunIMU(); });


    // build dynamic model
    quadruped_.BuildModel(fbmodel_);

    // init state estimator

    // sub cmd and register cmd handler
    driver_cmd_sub_ = this->create_subscription<sdrobot_api::msg::DriverCmd>(
        ros::kTopicCmd, 10, [this](const sdrobot_api::msg::DriverCmd::SharedPtr msg) {this->HandleDriverCmd(std::move(msg)); });

    // run main ctrl periodically
    dyn_timer_ = this->create_wall_timer(
        ctrldt::kDYN, [this]() { this->Run(); });

    return true;
  }

  bool Runner::Run()
  {
    // Run the state estimator step
    // stateEstimator->run();

    // Update the data from the robot
    auto leg_ctrl = quadruped_.GetLegCtrl();
    leg_ctrl.UpdateLegData(interface_->GetSPIData());
    leg_ctrl.ZeroLegCmd();
    leg_ctrl.SetLegEnabled(true);

    // Run ctrl
    // ctrl->runController();

    // Update cmd to the robot
    leg_ctrl.UpdateLegCmd(interface_->GetSPICmd());
    return true;
  }

  void Runner::HandleDriverCmd(const sdrobot_api::msg::DriverCmd::SharedPtr msg)
  {
    state_cmd_.UpdateCmd(
        msg->mv[0], msg->mv[1], msg->tr,
        msg->pa, static_cast<dynamics::Mode>(msg->mode));
  }

} // namespace sd::robot
