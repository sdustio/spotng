#include "controllers/mpc/cmpc.h"

namespace sdrobot::ctrl::mpc
{
  CMpc::CMpc(double _dt, int _iterations_between_mpc) : dt_(_dt),
                                                        iterationsBetweenMPC(_iterations_between_mpc)
  {
  }
}
