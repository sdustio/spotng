#include "sdrobot/leg.h"

namespace sdrobot::leg
{
  void Cmd::Zero()
  {
    tau_feed_forward.fill(0.);
    force_feed_forward.fill(0.);
    q_des.fill(0.);
    qd_des.fill(0.);
    p_des.fill(0.);
    v_des.fill(0.);
    kp_cartesian.fill(0.);
    kd_cartesian.fill(0.);
    kp_joint.fill(0.);
    kd_joint.fill(0.);
  }

  void Data::Zero()
  {
    q.fill(0.);
    qd.fill(0.);
    p.fill(0.);
    v.fill(0.);
    tau_estimate.fill(0.);
    J.fill(0.);
  }
}
