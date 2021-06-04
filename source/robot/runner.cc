#include "robot/runner.h"

namespace sd::robot
{
  using std::placeholders::_1;
  using namespace std::chrono_literals;

  Runner::Runner(InterfacePtr itf) :
    Node(ros::kNodeName, ros::kNodeNs),
    interface_(std::move(itf))
  {
    state_cmd_= std::make_unique<ctrl::StateCmd>(ctrldt::DYNsec);
    leg_ctrl_ = std::make_unique<ctrl::LegCtrl>();
    quadruped_ = std::make_unique<Quadruped>();
    fbmodel_ = std::make_unique<dynamics::FBModel>();
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
    quadruped_->BuildModel(fbmodel_);

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
    leg_ctrl_->UpdateData(interface_->GetSPIData());
    leg_ctrl_->ZeroCmd();
    leg_ctrl_->SetLegEnabled(true);

    // Run ctrl
    state_cmd_->CmdtoStateData();
    // ctrl->runController();

    // Update cmd to the robot
    leg_ctrl_->UpdateSPICmd(interface_->GetSPICmdForUpdate());
    return true;
  }

  void Runner::HandleDriverCmd(const sdrobot_api::msg::DriverCmd::SharedPtr msg)
  {
    state_cmd_->Update(
        msg->mv[0], msg->mv[1], msg->tr,
        msg->pa, static_cast<Mode>(msg->mode));
  }

} // namespace sd::robot
