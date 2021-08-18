# SdRobot

## Usage

```cpp
#include <memory>
#include <sdrobot/robot.h>

sdrobot::Options opts;
opts.drive_mode = sdrobot::DriveMode::kManual;
opts.ctrl_dt_sec = 0.002;
opts.act_itf_sec = 0.025;

sdrobot::interface::ActuatorInterface::SharedPtr act_itf;
/*....*/
sdrobot::Robot::Ptr robot;
sdrobot::Robot::Build(robot, opts, act_itf);

sdrobot::sensor::ImuData imu_data;
sdrobot::drive::DriveCmd drive_cmd;
/*...*/
robot->UpdateImu(imu_data);
robot->UpdateDriveCmd(drive_cmd);

robot->RunOnce();
```
