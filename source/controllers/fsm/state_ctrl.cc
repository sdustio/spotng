#include "sdrobot/controllers/fsm.h"

namespace sdrobot::ctrl::fsm
{
  StateCtrl::StateCtrl(const LegPtr &cleg, const robot::QuadrupedPtr &quad, const StateCmdPtr &cmd, const est::StateEstPtr &est)
      : leg_ctrl_(cleg), quad_(quad), state_cmd_(cmd), state_est_(est) {}

}
