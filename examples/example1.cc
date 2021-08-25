#include <iostream>

#include "echo_act_itf.h"
#include "sdrobot/robot.h"

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv) {
  using namespace sdrobot;

  Options opts;
  opts.drive_mode = DriveMode::kManual;
  opts.ctrl_dt_sec = 0.002;
  opts.act_itf_sec = 0.025;
  opts.jpos_init_sec = 0.1;

  interface::ActuatorInterface::SharedPtr itf =
      std::make_shared<EchoActuatorInterface>();
  /*....*/

  Robot::Ptr robot;
  Robot::Build(robot, opts, itf);

  sensor::ImuData imu_data;
  drive::Twist drive_twist;

  /*...*/
  robot->UpdateImu(imu_data);
  robot->UpdateDriveGait(drive::Gait::Trot);
  robot->UpdateDriveState(drive::State::Locomotion);
  drive_twist.lvel_x = 1.5;
  robot->UpdateDriveTwist(drive_twist);
  /*...*/

  unsigned cdt = 1000 * opts.ctrl_dt_sec;
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
      std::dynamic_pointer_cast<EchoActuatorInterface>(itf)->PrintCmd();
    }
  }

  return 0;
}
