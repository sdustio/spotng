# SdRobot

## Usage

```cpp
#include <memory>
#include <sdrobot/robot.h>

sdrobot::Options opts;
opts.drive_mode = sdrobot::DriveMode::kManual;
opts.ctrl_dt_sec = 0.001;

sdrobot::JointDriverPtr joint_driver;
/*....*/
sdrobot::RobotPtr robot = sdrobot::BuildRobot(opts, joint_driver);

sdrobot::ImuData imu_data;
sdrobot::DriveCmd drive_cmd;
/*...*/
robot->UpdateImu(imu_data);
robot->UpdateDriveCmd(drive_cmd);

robot->Run();
```
