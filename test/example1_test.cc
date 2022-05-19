#include <iostream>
#include <memory>

#include "testitf.h"
#include "sdengine/robot.h"

namespace sdengine::test {

int ctrl_iter = 0;

void RunRobot(RobotCtrl::Ptr const &robot, std::shared_ptr<LegImpl> const &legitf,
              std::shared_ptr<ImuImpl> const &imuitf, int const ctrl_dt, int const total_dt) {
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

  int ctrl_dt = 1000 * opts->ctrl_sec;

  auto leg_itf = std::make_shared<LegImpl>();
  auto imu_itf = std::make_shared<ImuImpl>();

  RobotCtrl::Ptr robot;
  RobotCtrl::Build(robot, opts, leg_itf, imu_itf);

  auto drive_ctrl = robot->GetDriveCtrl();

  // State Init
  RunRobot(robot, leg_itf, imu_itf, ctrl_dt, 510);

  // State Recovery Stand
  drive_ctrl->UpdateState(drive::State::RecoveryStand);
  RunRobot(robot, leg_itf, imu_itf, ctrl_dt, 2'000);

  // State Locomotion
  drive_ctrl->UpdateMode(drive::Mode::Manual);
  drive_ctrl->UpdateGait(drive::Gait::Trot);
  drive_ctrl->UpdateState(drive::State::Locomotion);
  drive::Twist drive_twist;
  drive_ctrl->UpdateTwist(drive_twist);
  RunRobot(robot, leg_itf, imu_itf, ctrl_dt, 1'000);
}

}  // namespace sdengine::test

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv) {
  std::cout << "Start Test!!!" << std::endl;
  sdengine::test::RunExample();
  std::cout << "End Test!!!" << std::endl;
  return 0;
}
