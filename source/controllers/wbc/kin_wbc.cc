#include "sdrobot/controllers/wbc.h"
#include "sdrobot/dynamics/math.h"

namespace sdrobot::ctrl::wbc
{

  KinWbc::KinWbc(int num_qdot)
      : num_qdot_(num_qdot), num_act_joint_(num_qdot - 6),
        I_mtx(MatrixX::Identity(num_qdot, num_qdot))
  {
  }

  bool KinWbc::FindConfiguration(const VectorX &curr_config,
                                 const std::vector<TaskPtr> &task_list, const std::vector<ContactPtr> &contact_list,
                                 VectorX &jpos_cmd, VectorX &jvel_cmd)
  {
    // Contact Jacobian Setup
    MatrixX Nc(num_qdot_, num_qdot_);
    Nc.setIdentity();
    if (contact_list.size() > 0)
    {
      MatrixX Jc, Jc_i;
      Jc = contact_list[0]->getContactJacobian();
      int num_rows = Jc.rows();

      int csize = contact_list.size();
      for (int i(1); i < csize; ++i)
      {
        Jc_i = contact_list[i]->getContactJacobian();
        int num_new_rows = Jc_i.rows();
        Jc.conservativeResize(num_rows + num_new_rows, num_qdot_);
        Jc.block(num_rows, 0, num_new_rows, num_qdot_) = Jc_i;
        num_rows += num_new_rows;
      }

      // Projection Matrix
      _BuildProjectionMatrix(Jc, Nc);
    }

    // First Task
    VectorX delta_q, qdot;
    MatrixX Jt, JtPre, JtPre_pinv, N_nx, N_pre;

    TaskPtr task = task_list[0];
    Jt = task->getTaskJacobian();
    JtPre = Jt * Nc;
    _PseudoInverse(JtPre, JtPre_pinv);

    delta_q = JtPre_pinv * (task->getPosError());
    qdot = JtPre_pinv * (task->getDesVel());

    VectorX prev_delta_q = delta_q;
    VectorX prev_qdot = qdot;

    _BuildProjectionMatrix(JtPre, N_nx);
    N_pre = Nc * N_nx;

    int tsize = task_list.size();
    for (int i(1); i < tsize; ++i)
    {
      task = task_list[i];

      Jt = task->getTaskJacobian();
      JtPre = Jt * N_pre;

      _PseudoInverse(JtPre, JtPre_pinv);
      delta_q =
          prev_delta_q + JtPre_pinv * (task->getPosError() - Jt * prev_delta_q);
      qdot = prev_qdot + JtPre_pinv * (task->getDesVel() - Jt * prev_qdot);

      // For the next task
      _BuildProjectionMatrix(JtPre, N_nx);
      N_pre *= N_nx;
      prev_delta_q = delta_q;
      prev_qdot = qdot;
    }
    for (int i(0); i < num_act_joint_; ++i)
    {
      jpos_cmd[i] = curr_config[i] + delta_q[i + 6];
      jvel_cmd[i] = qdot[i + 6];
    }
    return true;
  }

  void KinWbc::_BuildProjectionMatrix(const MatrixX &J, MatrixX &N)
  {
    MatrixX J_pinv;
    _PseudoInverse(J, J_pinv);
    N = I_mtx - J_pinv * J;
  }

  void KinWbc::_PseudoInverse(const MatrixX &J, MatrixX &Jinv)
  {
    dynamics::PseudoInverse(J, threshold_, Jinv);
  }
}
