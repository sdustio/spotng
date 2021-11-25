#include <iostream>
#include <memory>

#include "itf/impl.h"
#include "sdquadx/robot.h"

using namespace sdquadx;

void RunRobot(sdquadx::RobotCtrl::Ptr const &robot, std::shared_ptr<interface::LegImpl> const &legitf,
              std::shared_ptr<interface::ImuImpl> const &imuitf, int const cdt, int const itf_dt, int const iters) {
  for (int i = 0; i < iters; i++) {
    if (i % cdt == 0) {
      robot->RunOnce();
    }

    if (i % itf_dt == 0) {
      legitf->RunOnce();
      imuitf->RunOnce();
    }

    if (i % 50 == 0) {
      printf("\n===================\niter: %d\n", i);
      legitf->PrintCmd();
      // imuitf->PrintCmd();
    }
  }
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv) {
  auto opts = std::make_shared<sdquadx::Options>();
  opts->drive_mode = sdquadx::options::DriveMode::kManual;
  opts->ctrl_sec = 0.002;
  opts->jpos_init_sec = 0.1;
  opts->log_level = "debug";

  int cdt = 1000 * opts->ctrl_sec;
  int itf_dt = 25;

  auto leg_itf = std::make_shared<interface::LegImpl>();
  auto imu_itf = std::make_shared<interface::ImuImpl>();

  sdquadx::RobotCtrl::Ptr robot;
  sdquadx::RobotCtrl::Build(robot, opts, leg_itf, imu_itf);

  sdquadx::drive::Twist drive_twist;

  // State Init
  RunRobot(robot, leg_itf, imu_itf, cdt, itf_dt, 10'000);

  // State Recovery Stand
  robot->UpdateDriveState(sdquadx::drive::State::RecoveryStand);
  RunRobot(robot, leg_itf, imu_itf, cdt, itf_dt, 2000);
  RunRobot(robot, leg_itf, imu_itf, cdt, itf_dt, 100);
  RunRobot(robot, leg_itf, imu_itf, cdt, itf_dt, 2000);

  /*...*/
  robot->UpdateDriveGait(sdquadx::drive::Gait::Trot);
  robot->UpdateDriveState(sdquadx::drive::State::Locomotion);
  drive_twist.lvel_x = 1.5;
  robot->UpdateDriveTwist(drive_twist);
  /*...*/

  return 0;
}
