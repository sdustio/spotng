#include "itf/impl.h"

namespace sdquadx::interface {
bool LegImpl::ReadTo(sensor::LegData &data, [[maybe_unused]] int const leg) const {
  for (int leg = 0; leg < consts::model::kNumLeg; leg++) {
    // q: 关节角
    data.q[0] = data_.q_abad[leg];
    data.q[1] = data_.q_hip[leg];
    data.q[2] = data_.q_knee[leg];

    // qd 关节角速度
    data.qd[0] = data_.qd_abad[leg];
    data.qd[1] = data_.qd_hip[leg];
    data.qd[2] = data_.qd_knee[leg];
  }
  return true;
}
bool LegImpl::WriteFrom(LegCmds const &cmds) {
  for (int leg = 0; leg < consts::model::kNumLeg; leg++) {
    // tauFF 获得从控制器来的力矩
    auto const &tff = cmds[leg].tau;

    // set command: 命令设置 设置力矩
    cmd_.tau_abad_ff[leg] = tff[0];
    cmd_.tau_hip_ff[leg] = tff[1];
    cmd_.tau_knee_ff[leg] = tff[2];

    // joint space pd
    cmd_.kp_abad[leg] = cmds[leg].kp_joint[0];
    cmd_.kp_hip[leg] = cmds[leg].kp_joint[1];
    cmd_.kp_knee[leg] = cmds[leg].kp_joint[2];

    cmd_.kd_abad[leg] = cmds[leg].kd_joint[0];
    cmd_.kd_hip[leg] = cmds[leg].kd_joint[1];
    cmd_.kd_knee[leg] = cmds[leg].kd_joint[2];

    cmd_.q_des_abad[leg] = cmds[leg].q_des[0];
    cmd_.q_des_hip[leg] = cmds[leg].q_des[1];
    cmd_.q_des_knee[leg] = cmds[leg].q_des[2];

    cmd_.qd_des_abad[leg] = cmds[leg].qd_des[0];
    cmd_.qd_des_hip[leg] = cmds[leg].qd_des[1];
    cmd_.qd_des_knee[leg] = cmds[leg].qd_des[2];

    cmd_.flags[leg] = 1;
  }
  return true;
}

bool LegImpl::RunOnce() {
  data_.q_abad = cmd_.q_des_abad;
  data_.q_hip = cmd_.q_des_hip;
  data_.q_knee = cmd_.q_des_knee;
  data_.qd_abad = cmd_.qd_des_abad;
  data_.qd_hip = cmd_.qd_des_hip;
  data_.qd_knee = cmd_.qd_des_knee;
  data_.flags = cmd_.flags;
  data_.driver_status = 1;
  return true;
}

bool LegImpl::PrintCmd() const {
  printf("q_des_abad:\n");
  PrintArray4f(cmd_.q_des_abad);
  printf("q_des_hip:\n");
  PrintArray4f(cmd_.q_des_hip);
  printf("q_des_knee:\n");
  PrintArray4f(cmd_.q_des_knee);
  printf("qd_des_abad:\n");
  PrintArray4f(cmd_.qd_des_abad);
  printf("qd_des_hip:\n");
  PrintArray4f(cmd_.qd_des_hip);
  printf("qd_des_knee:\n");
  PrintArray4f(cmd_.qd_des_knee);
  printf("kp_abad:\n");
  PrintArray4f(cmd_.kp_abad);
  printf("kp_hip:\n");
  PrintArray4f(cmd_.kp_hip);
  printf("kp_knee:\n");
  PrintArray4f(cmd_.kp_knee);
  printf("kd_abad:\n");
  PrintArray4f(cmd_.kd_abad);
  printf("kd_hip:\n");
  PrintArray4f(cmd_.kd_hip);
  printf("kd_knee:\n");
  PrintArray4f(cmd_.kd_knee);
  printf("tau_abad_ff:\n");
  PrintArray4f(cmd_.tau_abad_ff);
  printf("tau_hip_ff:\n");
  PrintArray4f(cmd_.tau_hip_ff);
  printf("tau_knee_ff:\n");
  PrintArray4f(cmd_.tau_knee_ff);

  return true;
}

bool LegImpl::PrintArray4f(SdVector4f const &arr) const {
  for (auto &&i : arr) printf("%3.f\t", i);
  printf("\n");
  return true;
}

bool ImuImpl::ReadTo(sensor::ImuData &data) const {
  data = imu_;
  return true;
}

bool ImuImpl::RunOnce() {
  imu_.acc = {0., 0., 0.};
  imu_.gyro = {0., 0., 1.};
  imu_.quat = {0, 0, 0.247404, 0.9689124};
  return true;
}

}  // namespace sdquadx::interface
