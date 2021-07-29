#include "sdrobot/leg.h"

namespace sdrobot::leg
{
  namespace
  {
    constexpr std::array<double, 4> side_signs{-1.0, 1.0, -1.0, 1.0};
  }

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

  double GetSideSign(int leg)
  {
    return side_signs.at(leg);
  }
  void FlipWithSideSigns(SdVector3f &ret, const SdVector3f &v, int leg_id)
  {
    switch (leg_id)
    {
    case idx::fr:
      ret = {v[0], -v[1], v[2]};
      break;
    case idx::fl:
      ret = {v[0], v[1], v[2]};
      break;
    case idx::hr:
      ret = {-v[0], -v[1], v[2]};
      break;
    case idx::hl:
      ret = {-v[0], v[1], v[2]};
      break;
    default:
      throw std::runtime_error("Invalid leg id!");
    }
  }
}
