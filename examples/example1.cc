#include <iostream>
#include <memory>

#include "itf/echo_itf.h"
#include "sdquadx/robot.h"

void RunRobot(sdquadx::RobotCtrl::Ptr const &robot, sdquadx::interface::ActuatorInterface::SharedPtr const &itf,
              int const cdt, int const adt, int const iters) {
  for (int i = 0; i < iters; i++) {
    if (i % cdt == 0) {
      robot->RunOnce();
    }

    if (i % adt == 0) {
      itf->RunOnce();
    }

    if (i % 50 == 0) {
      printf("\n===================\niter: %d\n", i);
      std::dynamic_pointer_cast<EchoInterface>(itf)->PrintCmd();
    }
  }
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv) {
  auto opts = std::make_shared<sdquadx::Options>();
  opts->drive_mode = sdquadx::options::DriveMode::kManual;
  opts->ctrl_sec = 0.002;
  opts->act_itf_sec = 0.025;
  opts->jpos_init_sec = 0.1;
  opts->log_level = "debug";

  int cdt = 1000 * opts->ctrl_sec;
  int adt = 1000 * opts->act_itf_sec;

  sdquadx::interface::ActuatorInterface::SharedPtr itf = std::make_shared<EchoInterface>();

  sdquadx::RobotCtrl::Ptr robot;
  sdquadx::RobotCtrl::Build(robot, opts, itf);

  sdquadx::sensor::ImuData imu_data;
  sdquadx::drive::Twist drive_twist;

  // State Init
  RunRobot(robot, itf, cdt, adt, 10'000);

  // State Recovery Stand
  imu_data.quat = {0.0007963, 0.9999997, 0, 0};
  robot->UpdateImu(imu_data);
  robot->UpdateDriveState(sdquadx::drive::State::RecoveryStand);
  RunRobot(robot, itf, cdt, adt, 2000);
  RunRobot(robot, itf, cdt, adt, 100);
  imu_data.quat = {0, 0, 0, 1};
  robot->UpdateImu(imu_data);
  RunRobot(robot, itf, cdt, adt, 2000);

  /*...*/
  robot->UpdateDriveGait(sdquadx::drive::Gait::Trot);
  robot->UpdateDriveState(sdquadx::drive::State::Locomotion);
  drive_twist.lvel_x = 1.5;
  robot->UpdateDriveTwist(drive_twist);
  /*...*/

  return 0;
}
