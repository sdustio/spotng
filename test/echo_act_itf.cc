#include "echo_act_itf.h"

namespace sdrobot::interface
{
  ActuatorData const &EchoActuatorInterface::GetActuatorData() const { return data_; }
  ActuatorCmd &EchoActuatorInterface::GetActuatorCmdForUpdate() { return cmd_; }
  bool EchoActuatorInterface::UpdateActuatorCmd(ActuatorCmd const &cmd)
  {
    cmd_ = cmd;
    return true;
  }
  bool EchoActuatorInterface::Init() { return true; }
  bool EchoActuatorInterface::RunOnce()
  {
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
}
