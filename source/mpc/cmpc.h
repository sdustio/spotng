#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "spotng/drive.h"
#include "mpc/mpc.h"
#include "mpc/qp.h"
#include "skd/gait.h"

namespace spotng::mpc {

class CMpc : public Mpc {
 public:
  explicit CMpc(Options::ConstSharedPtr const &opts);
  bool RunOnce(wbc::InData &wbcdata, estimate::State const &estdata, skd::PredStanceVector const &st_states) override;

 private:
  Options::ConstSharedPtr const opts_;
  fpt_t const dt_;

  fpt_t x_integral_ = 0;
  SdVector3f rpy_integral_ = {};

  QpData qp_data_;
  QpSolver qp_solver_;
};
}  // namespace spotng::mpc
