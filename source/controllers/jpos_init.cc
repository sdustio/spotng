#include "sd/controllers/jpos_init.h"
#include "sd/robot/runner.h"

namespace sd::ctrl
{
  using robot::ctrldt::kDYNsec;

  JPosInit::JPosInit(double end_time) : end_time_(end_time), ini_jpos_(robot::QuadrupedProperties::num_act_joint)
  {
    target_jpos_ = {
        -0.6, -1.0, 2.7,
        0.6, -1.0, 2.7,
        -0.6, -1.0, 2.7,
        0.6, -1.0, 2.7};
    mid_jpos_ = {
        -1.8, 0., 2.7,
        1.8, 0., 2.7,
        -1.7, 0.5, 0.5,
        1.7, 0.5, 0.5};
  }

  bool JPosInit::IsInitialized(LegCtrlPtr &ctrl)
  {
    curr_time_ += kDYNsec;
    if(first_visit_){
      UpdateInitial(ctrl);
      first_visit_ = false;
    }

    if(curr_time_ < end_time_){
      //TODO trj
      return false;
    }
    return true;
  }

  void JPosInit::UpdateInitial(const LegCtrlPtr &ctrl)
  {
    ctrl->ZeroCmd();
  }

}
