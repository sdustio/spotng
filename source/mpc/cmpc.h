#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "mpc/mpc.h"
#include "mpc/qp.h"
#include "sdquadx/drive.h"
#include "skd/gait.h"

namespace sdquadx::mpc {

class CMpc : public Mpc {
 public:
  explicit CMpc(Options::ConstSharedPtr const &opts);
  bool RunOnce(wbc::InData &wbcdata, estimate::State const &estdata, skd::PredStanceVector const &st_states) override;

 private:
  Options::ConstSharedPtr const opts_;
  fpt_t const dt_;

  fpt_t x_comp_integral_ = 0;
  QpData qp_data_;
  QpSolver qp_solver_;
};
}  // namespace sdquadx::mpc
