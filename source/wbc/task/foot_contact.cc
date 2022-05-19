#include "wbc/task/foot_contact.h"

#include "utils/eigen.h"

namespace sdengine::wbc {

TaskFootContact::TaskFootContact(model::Quadruped::ConstSharedPtr const &quad, int leg)
    : Task({}, {}), mquad_(quad), leg_(leg) {}

bool TaskFootContact::UpdateTask([[maybe_unused]] estimate::State const &estate,
                                 [[maybe_unused]] SdVector3f const &x_des, [[maybe_unused]] SdVector3f const &xd_des,
                                 SdVector3f const &xdd_des) {
  auto const &dyndata = mquad_->GetDynamicsData();
  Jt_ = dyndata.Jc[leg_];
  Jtdqd_ = dyndata.Jcdqd[leg_];

  xdd_cmd_ = xdd_des;  // F_des

  return true;
}
}  // namespace sdengine::wbc
