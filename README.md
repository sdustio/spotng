# sdquadx

## Usage

```cpp
#include <memory>
#include <sdquadx/robot.h>

sdquadx::Options opts;
opts.drive_mode = sdquadx::DriveMode::kManual;
opts.ctrl_dt_sec = 0.002;
opts.act_itf_sec = 0.025;

sdquadx::interface::ActuatorInterface::SharedPtr act_itf;
/*....*/
sdquadx::Robot::Ptr robot;
sdquadx::Robot::Build(robot, opts, act_itf);

sdquadx::sensor::ImuData imu_data;
sdquadx::drive::DriveCmd drive_cmd;
/*...*/
robot->UpdateImu(imu_data);
robot->UpdateDriveCmd(drive_cmd);

robot->RunOnce();
```
