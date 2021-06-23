#include "robot/runner.h"

namespace sd::robot
{
  using std::placeholders::_1;
  using namespace std::chrono_literals;

  Runner::Runner(const InterfacePtr &itf) : Node(ros::kNodeName, ros::kNodeNs),
                                            interface_(itf)
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

    // sub imu and register imu handler
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        ros::kTopicIMU, 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg)
        { this->HandleIMU(msg); });

    // sub cmd and register cmd handler
    ctrl_state_cmd_ = std::make_shared<ctrl::StateCmd>(ctrlparams::kCtrlsec);
    cmd_sub_ = this->create_subscription<sdrobot_interfaces::msg::Cmd>(
        ros::kTopicCmd, 10, [this](const sdrobot_interfaces::msg::Cmd::SharedPtr msg)
        { this->HandleCmd(msg); });

    // build dynamic model
    quadruped_ = std::make_shared<Quadruped>();
    fbmodel_ = quadruped_->BuildModel();

    // init state estimator
    contact_phase_ << 0.5, 0.5, 0.5, 0.5;
    est_contact_ = std::make_shared<est::Contact>();
    est_orientation_ = std::make_shared<est::Orientation>();
    est_pos_vel_ = std::make_shared<est::PosVel>();

    // init ctrls
    ctrl_leg_ = std::make_shared<ctrl::Leg>();
    ctrl_jpos_init_ = std::make_shared<ctrl::JPosInit>();
    ctrl_gait_skd_ = std::make_shared<ctrl::GaitSkd>();

    // run main ctrl periodically
    ctrl_timer_ = this->create_wall_timer(
        ctrlparams::kCtrldt, [this]()
        { this->Run(); });

    return true;
  }

  bool Runner::Run()
  {
    // Update the data from the robot
    ctrl_leg_->UpdateDatas(interface_->GetSPIData());
    ctrl_leg_->ZeroCmd();
    ctrl_leg_->SetLegEnabled(true);

    // Run the state estimator step
    est_contact_->Run(est_ret_, contact_phase_);
    est_orientation_->Run(est_ret_, imu_data_);
    est_pos_vel_->Run(est_ret_, ctrl_leg_->GetDatas(), quadruped_);

    if (ctrl_jpos_init_->IsInitialized(ctrl_leg_))
    {
      // Run ctrl
      ctrl_gait_skd_->Step();
      ctrl_state_cmd_->CmdtoStateData();
      // TODO ctrl->runController();
    }

    // Update cmd to the robot
    ctrl_leg_->UpdateSPICmd(interface_->GetSPICmdForUpdate());
    // TODO publish motion data
    return true;
  }

  void Runner::HandleCmd(const sdrobot_interfaces::msg::Cmd::SharedPtr &msg)
  {
    ctrl_state_cmd_->Update(
        msg->linear_velocity.x, msg->linear_velocity.y,
        msg->angular_velocity.z, msg->rpy_angle.y,
        static_cast<Mode>(msg->mode));
  }

  void Runner::HandleIMU(const sensor_msgs::msg::Imu::SharedPtr &msg)
  {
    imu_data_.acc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    imu_data_.gyro << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    imu_data_.quat << msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;
  }

} // namespace sd::robot
