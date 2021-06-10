#include "robot/runner.h"

namespace sd::robot
{
  using std::placeholders::_1;
  using namespace std::chrono_literals;

  Runner::Runner(InterfacePtr itf) : Node(ros::kNodeName, ros::kNodeNs),
                                     interface_(std::move(itf))
  {
    Init();
  }

  bool Runner::Init()
  {
    // interface init & run periodically
    interface_->Init();
    spi_timer_ = this->create_wall_timer(
        ctrlparams::kSPIdt, [this]()
        { this->interface_->RunSPI(); });
    imu_timer_ = this->create_wall_timer(
        ctrlparams::kIMUdt, [this]()
        { this->interface_->RunIMU(); });

    // build dynamic model
    quadruped_ = std::make_unique<Quadruped>();
    fbmodel_ = quadruped_->BuildModel();

    // init ctrls
    leg_ctrl_ = std::make_unique<ctrl::LegCtrl>();
    jpos_ctrl_ = std::make_unique<ctrl::JPosInit>();

    // init state estimator
    // TODO initializeStateEstimator

    // sub cmd and register cmd handler
    state_cmd_ = std::make_unique<ctrl::StateCmd>(ctrlparams::kCtrlsec);
    driver_cmd_sub_ = this->create_subscription<sdrobot_api::msg::DriverCmd>(
        ros::kTopicCmd, 10, [this](const sdrobot_api::msg::DriverCmd::SharedPtr msg)
        { this->HandleDriverCmd(std::move(msg)); });

    // run main ctrl periodically
    dyn_timer_ = this->create_wall_timer(
        ctrlparams::kCtrldt, [this]()
        { this->Run(); });

    return true;
  }

  bool Runner::Run()
  {
    // Run the state estimator step
    // TODO stateEstimator->run();

    // Update the data from the robot
    leg_ctrl_->UpdateData(interface_->GetSPIData());
    leg_ctrl_->ZeroCmd();
    leg_ctrl_->SetLegEnabled(true);

    if (jpos_ctrl_->IsInitialized(leg_ctrl_))
    {
      // Run ctrl
      state_cmd_->CmdtoStateData();
      // TODO ctrl->runController();
    }

    // Update cmd to the robot
    leg_ctrl_->UpdateSPICmd(interface_->GetSPICmdForUpdate());
    // TODO publish motion data
    return true;
  }

  void Runner::HandleDriverCmd(const sdrobot_api::msg::DriverCmd::SharedPtr msg)
  {
    state_cmd_->Update(
        msg->mv[0], msg->mv[1], msg->tr,
        msg->pa, static_cast<Mode>(msg->mode));
  }

} // namespace sd::robot
