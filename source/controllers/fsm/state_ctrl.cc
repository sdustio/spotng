#include "sd/controllers/fsm.h"

namespace sd::ctrl::fsm
{
  StateCtrl::StateCtrl(LegPtr &cleg, const StateCmdPtr &cmd, const est::StateEstPtr &est)
      : leg_ctrl_(cleg), state_cmd_(cmd), state_est_(est) {}

}
