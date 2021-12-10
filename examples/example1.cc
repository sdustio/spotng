#include <iostream>
#include <memory>

#include "itf/impl.h"
#include "sdquadx/robot.h"

namespace sdquadx {
void RunRobot(RobotCtrl::Ptr const &robot, std::shared_ptr<interface::LegImpl> const &legitf,
              std::shared_ptr<interface::ImuImpl> const &imuitf, int const ctrl_dt, int const itf_dt, int const iters) {
  for (int i = 0; i < iters; i++) {
    if (i % ctrl_dt == 0) {
      robot->RunOnce();
    }

    if (i % itf_dt == 0) {
      legitf->RunOnce();
      imuitf->RunOnce();
    }
  }
}

void RunExample() {
  auto opts = std::make_shared<Options>();
  opts->ctrl_sec = 0.002;
  opts->jpos_init_sec = 0.1;
  opts->log_level = "debug";

  int ctrl_dt = 1000 * opts->ctrl_sec;
  int itf_dt = 25;

  auto leg_itf = std::make_shared<interface::LegImpl>();
  auto imu_itf = std::make_shared<interface::ImuImpl>();

  RobotCtrl::Ptr robot;
  RobotCtrl::Build(robot, opts, leg_itf, imu_itf);

  auto drive_ctrl = robot->GetDriveCtrl();

  // State Init
  RunRobot(robot, leg_itf, imu_itf, ctrl_dt, itf_dt, 10'000);

  // State Recovery Stand
  drive_ctrl->UpdateState(drive::State::RecoveryStand);
  RunRobot(robot, leg_itf, imu_itf, ctrl_dt, itf_dt, 2000);
  RunRobot(robot, leg_itf, imu_itf, ctrl_dt, itf_dt, 100);
  RunRobot(robot, leg_itf, imu_itf, ctrl_dt, itf_dt, 2000);

  drive_ctrl->UpdateMode(drive::Mode::Manual);
  drive_ctrl->UpdateGait(drive::Gait::Trot);
  drive_ctrl->UpdateState(drive::State::Locomotion);
  drive::Twist drive_twist;
  drive_ctrl->UpdateTwist(drive_twist);
  RunRobot(robot, leg_itf, imu_itf, ctrl_dt, itf_dt, 10000);
}

}  // namespace sdquadx

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv) {
  sdquadx::RunExample();
  return 0;
}
