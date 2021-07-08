#include "sd/controllers/wbc.h"

namespace sd::ctrl::wbc
{

  KinWbc::KinWbc(size_t num_qdot) {}
  bool KinWbc::FindConfiguration(const VectorXd &curr_config,
                                 const std::vector<TaskPtr> &task_list, const std::vector<ContactPtr> &contact_list,
                                 VectorXd &jpos_cmd, VectorXd &jvel_cmd) { return true; }
}
