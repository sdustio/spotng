#include "sd/controllers/wbc.h"

namespace sd::ctrl::wbc
{
  Wbic::Wbic(size_t num_qdot, double weight) : _W_floating(VectorXd::Constant(6, weight)),
                                               _W_rf(VectorXd::Constant(12, 1.))
  {
  }

  void Wbic::UpdateSetting(const MatrixXd &A, const MatrixXd &Ainv,
                           const VectorXd &cori, const VectorXd &grav)
  {
    A_ = A;
    Ainv_ = Ainv;
    cori_ = cori;
    grav_ = grav;
    b_updatesetting_ = true;
  }

  void Wbic::MakeTorque(VectorXd &cmd, const std::vector<TaskPtr> &task_list, const std::vector<ContactPtr> &contact_list)
  {
  }
}
