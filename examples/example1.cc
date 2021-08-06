#include <iostream>

#include "sdrobot/robot.h"
#include "echo_act_itf.h"

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
  using namespace sdrobot;

  Options opts;
  opts.drive_mode = DriveMode::kManualAll;
  opts.ctrl_dt_sec = 0.002;
  opts.act_itf_sec = 0.025;
  opts.jpos_init_sec = 0.05;

  interface::ActuatorInterface::SharedPtr itf = std::make_shared<EchoActuatorInterface>();
  /*....*/

  Robot::Ptr robot;
  Robot::Build(robot, opts, itf);

  sensor::ImuData imu_data;
  drive::DriveCmd drive_cmd;

  /*...*/
  drive_cmd.gait = drive::Gait::Trot;
  drive_cmd.state = drive::State::Locomotion;
  drive_cmd.move_x = 3.;
  /*...*/

  robot->UpdateImu(imu_data);
  robot->UpdateDriveCmd(drive_cmd);

  for (size_t i = 0; i < 100; i++)
  {
    printf("\n===================\niter: %zu\n", i);
    robot->RunOnce();
    itf->RunOnce();
    if (i % 10 == 0)
    {
      std::dynamic_pointer_cast<EchoActuatorInterface>(itf)->PrintCmd();
    }

  }

  return 0;
}
