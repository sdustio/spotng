#include <iostream>
#include <memory>

#include "itf/impl.h"
#include "sdquadx/robot.h"

namespace sdquadx::test {

int ctrl_iter = 0;

void RunRobot(RobotCtrl::Ptr const &robot, std::shared_ptr<interface::LegImpl> const &legitf,
              std::shared_ptr<interface::ImuImpl> const &imuitf, int const ctrl_dt, int const total_dt) {
  for (int i = 0; i < total_dt; i++) {
    if (i % ctrl_dt == 0) {
      robot->RunOnce();
      ctrl_iter++;
      printf("!!!![Iteration %d]\n", ctrl_iter);
    }
    legitf->RunOnce();
    imuitf->RunOnce();
  }
}

void RunExample() {
  auto opts = std::make_shared<Options>();
  opts->ctrl_sec = 0.002;
  opts->jpos_init_sec = 0.1;
  opts->log_level = "debug";

  int ctrl_dt = 1000 * opts->ctrl_sec;

  auto leg_itf = std::make_shared<interface::LegImpl>();
  auto imu_itf = std::make_shared<interface::ImuImpl>();

  RobotCtrl::Ptr robot;
  RobotCtrl::Build(robot, opts, leg_itf, imu_itf);

  auto drive_ctrl = robot->GetDriveCtrl();

  // State Init
  RunRobot(robot, leg_itf, imu_itf, ctrl_dt, 10'000);

  // State Recovery Stand
  drive_ctrl->UpdateState(drive::State::RecoveryStand);
  RunRobot(robot, leg_itf, imu_itf, ctrl_dt, 10'000);

  // State Locomotion
  drive_ctrl->UpdateMode(drive::Mode::Manual);
  drive_ctrl->UpdateGait(drive::Gait::Trot);
  drive_ctrl->UpdateState(drive::State::Locomotion);
  drive::Twist drive_twist;
  drive_ctrl->UpdateTwist(drive_twist);
  RunRobot(robot, leg_itf, imu_itf, ctrl_dt, 20'000);
}

}  // namespace sdquadx::test
