#include "sdrobot/interface.h"

using namespace sdrobot::interface;

class EchoActuatorInterface : public ActuatorInterface
{
public:
  ActuatorData const &GetActuatorData() const override;
  ActuatorCmd &GetActuatorCmdForUpdate() override;
  bool UpdateActuatorCmd(ActuatorCmd const &cmd) override;
  bool RunOnce() override;

private:
  bool PrintCmd() const;
  bool PrintArray4f(sdrobot::SdVector4f const &) const;

  ActuatorData data_;
  ActuatorCmd cmd_;
};
