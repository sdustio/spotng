#include "itf/echo_itf.h"

ActuatorData const &EchoInterface::GetActuatorData() const { return data_; }

ActuatorCmd &EchoInterface::GetActuatorCmdForUpdate() { return cmd_; }

bool EchoInterface::UpdateActuatorCmd(ActuatorCmd const &cmd) {
  cmd_ = cmd;
  return true;
}

bool EchoInterface::RunOnce() {
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

bool EchoInterface::PrintCmd() const {
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

bool EchoInterface::PrintArray4f(SdVector4f const &arr) const {
  for (auto &&i : arr) printf("%3.f\t", i);
  printf("\n");
  return true;
}
