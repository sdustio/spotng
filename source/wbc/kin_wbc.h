#pragma once

#include "wbc/task.h"
#include "wbc/contact.h"
#include "wbc/wbc.h"
#include "eigen.h"

namespace sdrobot::wbc
{
  class KinWbc
  {
  public:
    using Ptr = std::unique_ptr<KinWbc>;
    using SharedPtr = std::shared_ptr<KinWbc>;

    bool FindConfiguration(SdVector12f &jpos_cmd, SdVector12f &jvel_cmd, SdVector12f const &curr_config,
                           std::vector<Task::ConstSharedPtr> const &task_list, std::vector<Contact::ConstSharedPtr> const &contact_list);

  private:
    bool _PseudoInverse(MatrixX &ret, MatrixX const &J, fpt_t threshold = 0.001);
    bool _BuildProjectionMatrix(MatrixX &ret, MatrixX const &J);
  };
}
