#pragma once

#include <memory>

#include "spotng/model.h"

namespace spotng::wbc {

class Task {
 public:
  using Ptr = std::unique_ptr<Task>;
  using SharedPtr = std::shared_ptr<Task>;
  using ConstSharedPtr = std::shared_ptr<Task const>;

  Task(SdVector3f const &kp, SdVector3f const &kd) : Kp_(kp), Kd_(kd) {
    Jt_.fill(0.);
    Jtdqd_.fill(0.);
  }

  virtual bool UpdateTask(estimate::State const &estate, SdVector3f const &x_des, SdVector3f const &xd_des,
                          SdVector3f const &xdd_des) = 0;

  std::array<fpt_t, 3 * consts::model::kDimConfig> const &GetTaskJacobian() const { return Jt_; }
  SdVector3f const &GetTaskJacobianDotQdot() const { return Jtdqd_; }
  SdVector3f const &GetXError() const { return x_err_; }
  SdVector3f const &GetXdDes() const { return xd_des_; }
  SdVector3f const &GetXddCmd() const { return xdd_cmd_; }

 protected:
  std::array<fpt_t, 3 * consts::model::kDimConfig> Jt_;
  SdVector3f Jtdqd_;

  SdVector3f x_err_;
  SdVector3f xd_des_;
  SdVector3f xdd_cmd_;

  SdVector3f Kp_, Kd_;
};
}  // namespace spotng::wbc
