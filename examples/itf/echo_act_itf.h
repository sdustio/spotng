#include "sdrobot/interface.h"

using sdrobot::SdVector4f;
using sdrobot::interface::ActuatorCmd;
using sdrobot::interface::ActuatorData;
using sdrobot::interface::ActuatorInterface;

class EchoActuatorInterface : public ActuatorInterface {
 public:
  ActuatorData const &GetActuatorData() const override;
  ActuatorCmd &GetActuatorCmdForUpdate() override;
  bool UpdateActuatorCmd(ActuatorCmd const &cmd) override;
  bool RunOnce() override;
  bool PrintCmd() const;

 private:
  bool PrintArray4f(SdVector4f const &) const;

  ActuatorData data_;
  ActuatorCmd cmd_;
};
