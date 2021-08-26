#include "sdquadx/interface.h"

using sdquadx::SdVector4f;
using sdquadx::interface::ActuatorCmd;
using sdquadx::interface::ActuatorData;
using sdquadx::interface::ActuatorInterface;

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
