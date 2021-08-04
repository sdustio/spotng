#include "wbc/kin_wbc.h"
#include "math/algebra.h"

namespace sdrobot::wbc
{
  bool KinWbc::FindConfiguration(SdVector12f const &curr_config,
                                 std::vector<Task::Ptr> const &task_list, std::vector<Contact::Ptr> const &contact_list,
                                 SdVector12f &jpos_cmd, SdVector12f &jvel_cmd)
  {
    // Contact Jacobian Setup
    MatrixX Nc(params::model::dim_config, params::model::dim_config);
    Nc.setIdentity();

    if (contact_list.size() > 0)
    {
      int num_rows = 0;
      MatrixX Jc;
      for (size_t i = 0; i < contact_list.size(); ++i)
      {
        Eigen::Map<Eigen::Matrix<fpt_t, 3, params::model::dim_config> const> _Jc(
            contact_list[i]->GetContactJacobian().data());
        auto num_new_rows = _Jc.rows(); //3
        Jc.conservativeResize(num_rows + num_new_rows, params::model::dim_config);
        Jc.block(num_rows, 0, num_new_rows, params::model::dim_config) = _Jc;
        num_rows += num_new_rows;
      }

      // Projection Matrix
      _BuildProjectionMatrix(Nc, Jc);
    }

    Vector18 delta_q = Vector18::Zero();
    Vector18 qdot = Vector18::Zero();
    MatrixX Jt, Jt_pinv, N_nx;
    for (size_t i = 0; i < task_list.size(); ++i)
    {
      auto const &task = task_list[i];
      Eigen::Map<Eigen::Matrix<fpt_t, 3, params::model::dim_config> const> _Jt(
          task->GetTaskJacobian().data());
      Jt = _Jt * Nc;
      _PseudoInverse(Jt_pinv, Jt);

      delta_q = delta_q + Jt_pinv * (ToConstEigenTp(task->GetPosError()) - _Jt * delta_q);
      qdot = qdot + Jt_pinv * (ToConstEigenTp(task->GetDesVel()) - _Jt * qdot);

      _BuildProjectionMatrix(N_nx, Jt);
      Nc = Nc * N_nx;
    }

    for (int i(0); i < params::model::num_act_joint; ++i)
    {
      jpos_cmd[i] = curr_config[i] + delta_q[i + 6];
      jvel_cmd[i] = qdot[i + 6];
    }
    return true;
  }

  bool KinWbc::_BuildProjectionMatrix(MatrixX &ret, MatrixX const &J)
  {
    MatrixX J_pinv;
    _PseudoInverse(J_pinv, J);
    ret = mat18_t::Identity() - J_pinv * J;
    return true;
  }

  bool KinWbc::_PseudoInverse(MatrixX &ret, MatrixX const &J, fpt_t threshold)
  {
    return math::PseudoInverse(ret, J, threshold);
  }
}
