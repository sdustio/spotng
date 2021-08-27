#include <iostream>

#include "itf/echo_itf.h"
#include "sdquadx/robot.h"

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv) {
  sdquadx::Options opts;
  opts.drive_mode = sdquadx::DriveMode::kManual;
  opts.ctrl_sec = 0.002;
  opts.act_itf_sec = 0.025;
  opts.jpos_init_sec = 0.1;

  sdquadx::interface::ActuatorInterface::SharedPtr itf = std::make_shared<EchoInterface>();
  /*....*/

  sdquadx::robot::RobotCtrl::Ptr robot;
  sdquadx::robot::RobotCtrl::Build(robot, opts, itf);

  sdquadx::sensor::ImuData imu_data;
  sdquadx::drive::Twist drive_twist;

  /*...*/
  robot->UpdateImu(imu_data);
  robot->UpdateDriveGait(sdquadx::drive::Gait::Trot);
  robot->UpdateDriveState(sdquadx::drive::State::Locomotion);
  drive_twist.lvel_x = 1.5;
  robot->UpdateDriveTwist(drive_twist);
  /*...*/

  unsigned cdt = 1000 * opts.ctrl_sec;
  unsigned adt = 1000 * opts.act_itf_sec;

  for (size_t i = 0; i < 10'000; i++) {
    if (i % cdt == 0) {
      robot->RunOnce();
    }

    if (i % adt == 0) {
      itf->RunOnce();
    }

    if (i % 50 == 0) {
      printf("\n===================\niter: %zu\n", i);
      std::dynamic_pointer_cast<EchoInterface>(itf)->PrintCmd();
    }
  }

  return 0;
}
