#include "sd/controllers/fsm.h"

namespace sd::ctrl::fsm
{
  StateCtrl::StateCtrl(const LegPtr &cleg, const robot::QuadrupedPtr &quad, const StateCmdPtr &cmd, const est::StateEstPtr &est)
      : leg_ctrl_(cleg), quad_(quad), state_cmd_(cmd), state_est_(est) {}

}
