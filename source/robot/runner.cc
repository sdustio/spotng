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
    ctrl_leg_ = std::make_unique<ctrl::Leg>();
    ctrl_jpos_init_ = std::make_unique<ctrl::JPosInit>(3.); //endtime = 3.0

    // init state estimator
    est_ori_ = std::make_unique<estimators::Orientation>();

    // sub cmd and register cmd handler
    ctrl_state_cmd_ = std::make_unique<ctrl::StateCmd>(ctrlparams::kCtrlsec);
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
    est_ori_->Run(est_ret_, interface_->GetIMUData());

    // Update the data from the robot
    ctrl_leg_->UpdateData(interface_->GetSPIData());
    ctrl_leg_->ZeroCmd();
    ctrl_leg_->SetLegEnabled(true);

    if (ctrl_jpos_init_->IsInitialized(ctrl_leg_))
    {
      // Run ctrl
      ctrl_state_cmd_->CmdtoStateData();
      // TODO ctrl->runController();
    }

    // Update cmd to the robot
    ctrl_leg_->UpdateSPICmd(interface_->GetSPICmdForUpdate());
    // TODO publish motion data
    return true;
  }

  void Runner::HandleDriverCmd(const sdrobot_api::msg::DriverCmd::SharedPtr msg)
  {
    ctrl_state_cmd_->Update(
        msg->mv[0], msg->mv[1], msg->tr,
        msg->pa, static_cast<Mode>(msg->mode));
  }

} // namespace sd::robot
