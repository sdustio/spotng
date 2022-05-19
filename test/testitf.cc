#include "testitf.h"

namespace sdengine::test {

bool LegImpl::ReadTo(sensor::LegDatas &data) const {
  for (int leg = 0; leg < consts::model::kNumLeg; leg++) {
    // q: 关节角
    data[leg].q[0] = data_.q_abad[leg];
    data[leg].q[1] = data_.q_hip[leg];
    data[leg].q[2] = data_.q_knee[leg];

    // qd 关节角速度
    data[leg].qd[0] = data_.qd_abad[leg];
    data[leg].qd[1] = data_.qd_hip[leg];
    data[leg].qd[2] = data_.qd_knee[leg];
  }
  return true;
}
bool LegImpl::WriteFrom(interface::LegCmds const &cmds) {
  for (int leg = 0; leg < consts::model::kNumLeg; leg++) {
    cmd_.tau_abad_ff[leg] = cmds[leg].tau[0];
    cmd_.tau_hip_ff[leg] = cmds[leg].tau[1];
    cmd_.tau_knee_ff[leg] = cmds[leg].tau[2];

    // joint space pd
    cmd_.kp_abad[leg] = cmds[leg].kp[0];
    cmd_.kp_hip[leg] = cmds[leg].kp[1];
    cmd_.kp_knee[leg] = cmds[leg].kp[2];

    cmd_.kd_abad[leg] = cmds[leg].kd[0];
    cmd_.kd_hip[leg] = cmds[leg].kd[1];
    cmd_.kd_knee[leg] = cmds[leg].kd[2];

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

bool ImuImpl::ReadTo(sensor::ImuData &data) const {
  data = imu_;
  return true;
}

bool ImuImpl::RunOnce() {
  imu_.acc = {0., 0., 9.81};
  return true;
}

}  // namespace sdengine::test
