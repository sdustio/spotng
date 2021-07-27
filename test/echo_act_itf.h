#include "sdrobot/interface.h"

namespace sdrobot::interface
{
  class EchoActuatorInterface : ActuatorInterface
  {
  public:
    ActuatorData const &GetActuatorData() const override;
    ActuatorCmd &GetActuatorCmdForUpdate() override;
    bool UpdateActuatorCmd(ActuatorCmd const &cmd) override;
    bool Init() override;
    bool RunOnce() override;

  private:
    ActuatorData data_;
    ActuatorCmd cmd_;
  };
}
