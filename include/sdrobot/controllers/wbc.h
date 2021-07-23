#pragma once

#include <memory>
#include <array>
#include <vector>

#include <eiquadprog/eiquadprog-fast.hpp>

#include "sdrobot/estimators/state_est.h"
#include "sdrobot/dynamics/fb_model.h"
#include "sdrobot/controllers/leg.h"
#include "sdrobot/controllers/state_cmd.h"


namespace sdrobot::ctrl
{

  namespace wbc
  {
    class Task
    {
    public:
      Task(int dim, const dynamics::FBModelPtr &model)
          : dim_task_(dim),
            robot_sys_(model),
            op_cmd_(dim),
            pos_err_(dim),
            vel_des_(dim),
            acc_des_(dim),
            Kp_(dim),
            Kd_(dim) {}

      bool UpdateTask(const Vector3 &pos_des, const Vector3 &vel_des,
                      const Vector3 &acc_des)
      {
        _UpdateTaskJacobian();
        _UpdateTaskJDotQdot();
        _UpdateCommand(pos_des, vel_des, acc_des);
        _AdditionalUpdate();
        b_set_task_ = true;
        return true;
      }

      const VectorX &GetCommand() { return op_cmd_; }
      const MatrixX &GetTaskJacobian() { return Jt_; }
      const VectorX &GetTaskJacobianDotQdot() { return JtDotQdot_; }

      void UpdateKp(int idx, double v) { Kp_(idx) = v; }

      const VectorX &GetKp() { return Kp_; }

      void UpdateKd(int idx, double v) { Kd_(idx) = v; }

      const VectorX &GetKd() { return Kd_; }

      const VectorX &GetPosError() { return pos_err_; }
      const VectorX &GetDesVel() { return vel_des_; }
      const VectorX &GetDesAcc() { return acc_des_; }

    protected:
      // Update op_cmd_
      virtual bool _UpdateCommand(const Vector3 &pos_des, const Vector3 &vel_des,
                                  const Vector3 &acc_des) = 0;
      // Update Jt_
      virtual bool _UpdateTaskJacobian() = 0;
      // Update JtDotQdot_
      virtual bool _UpdateTaskJDotQdot() = 0;
      // Additional Update (defined in child classes)
      virtual bool _AdditionalUpdate() = 0;

      bool b_set_task_ = false;
      int dim_task_;

      const dynamics::FBModelPtr robot_sys_;

      VectorX JtDotQdot_;
      MatrixX Jt_;

      VectorX op_cmd_;
      VectorX pos_err_;
      VectorX vel_des_, acc_des_;

      VectorX Kp_kin_;
      VectorX Kp_, Kd_;
    };

    using TaskPtr = std::shared_ptr<Task>;

    class Contact
    {
    public:
      Contact(int dim) : dim_contact_(dim)
      {
        idx_Fz_ = dim - 1; // because normally (tau_x,y,z , linear_x,y,z)
        Fr_des_ = VectorX::Zero(dim);
      }

      int GetDim() const { return dim_contact_; }
      int GetDimRFConstraint() const { return Uf_.rows(); }
      int GetFzIndex() const { return idx_Fz_; }

      const VectorX &GetRFDesired() { return Fr_des_; }
      void SetRFDesired(const VectorX &Fr_des) { Fr_des_ = Fr_des; }
      bool UpdateContact()
      {
        _UpdateJc();
        _UpdateJcDotQdot();
        _UpdateUf();
        _UpdateInequalityVector();
        b_set_contact_ = true;
        return true;
      }
      const MatrixX &GetContactJacobian() { return Jc_; }
      const VectorX &GetJcDotQdot() { return JcDotQdot_; }
      const MatrixX &GetRFConstraintMtx() { return Uf_; }
      const VectorX &GetRFConstraintVec() { return ieq_vec_; }

    protected:
      virtual bool _UpdateJc() = 0;
      virtual bool _UpdateJcDotQdot() = 0;
      virtual bool _UpdateUf() = 0;
      virtual bool _UpdateInequalityVector() = 0;

      int idx_Fz_;
      MatrixX Uf_;
      VectorX ieq_vec_;

      VectorX Fr_des_;

      MatrixX Jc_;
      VectorX JcDotQdot_;

      int dim_contact_;
      bool b_set_contact_ = false;
    };

    using ContactPtr = std::shared_ptr<Contact>;

    class Wbic
    {
    public:
      Wbic(int num_qdot, double weight);

      void UpdateSetting(const MatrixX &A, const MatrixX &Ainv,
                         const VectorX &cori, const VectorX &grav);

      void MakeTorque(VectorX &cmd, const std::vector<TaskPtr> &task_list, const std::vector<ContactPtr> &contact_list);

    private:
      void _ContactBuilding(const std::vector<ContactPtr> &contact_list);
      void _SetEqualityConstraint(const VectorX &qddot);
      void _SetInEqualityConstraint();

      void _SetOptimizationSize(const std::vector<ContactPtr> &contact_list);
      void _SetCost();
      void _GetSolution(const VectorX &qddot, VectorX &cmd);

      void _WeightedInverse(const MatrixX &J, const MatrixX &Winv, MatrixX &Jinv,
                            double threshold = 0.0001);

      int _SolveQP();

      int num_act_joint_;
      int num_qdot_;

      MatrixX Sa_; // Actuated joint
      MatrixX Sv_; // Virtual joint

      MatrixX A_;
      MatrixX Ainv_;
      VectorX cori_;
      VectorX grav_;

      bool b_updatesetting_ = false;

      int dim_opt_;
      int dim_eq_cstr_; // equality constraints

      int dim_rf_;
      int dim_Uf_;

      int dim_floating_ = 6;

      eiquadprog::solvers::EiquadprogFast qp_;

      VectorX z_;
      // Cost
      MatrixX G_;
      VectorX g0_;

      // Equality
      MatrixX CE_;
      VectorX ce0_;

      // Inequality
      MatrixX CI_;
      VectorX ci0_;

      MatrixX eye_;

      MatrixX Uf_;
      VectorX Uf_ieq_vec_;

      MatrixX Jc_;
      VectorX JcDotQdot_;
      VectorX Fr_des_;

      // Extra data
      // Output
      // VectorX _opt_result;
      // VectorX _qddot;
      // VectorX _Fr;

      // Input
      VectorX W_floating_;
      VectorX W_rf_;
    };

    class KinWbc
    {
    public:
      KinWbc(int num_qdot);
      bool FindConfiguration(const VectorX &curr_config,
                             const std::vector<TaskPtr> &task_list, const std::vector<ContactPtr> &contact_list,
                             VectorX &jpos_cmd, VectorX &jvel_cmd);

    private:
      void _PseudoInverse(const MatrixX &J, MatrixX &Jinv);
      void _BuildProjectionMatrix(const MatrixX &J, MatrixX &N);

      double threshold_ = 0.001;
      int num_qdot_;
      int num_act_joint_;
      MatrixX I_mtx_;
    };

  } // namespace wbc

  struct WbcData
  {
    Vector3 p_body_des;
    Vector3 v_body_des;
    Vector3 a_body_des;
    Vector3 p_body_rpy_des;
    Vector3 vbody_ori_des;

    std::array<Vector3, 4> p_foot_des;
    std::array<Vector3, 4> v_foot_des;
    std::array<Vector3, 4> a_foot_des;

    std::array<Vector3, 4> Fr_des;

    Vector4 contact_state;
  };

  class Wbc
  {
  public:
    Wbc(const dynamics::FBModelPtr &model, double weight = 0.1);
    void Run(const WbcData &, const StateCmdPtr &, const est::StateEstPtr &, LegPtr &);

  private:
    void _UpdateModel(const est::StateData &, const robot::leg::Datas &);
    void _ComputeWBC();
    void _UpdateLegCMD(LegPtr &, const StateCmdPtr &);
    void _ContactTaskUpdate(const WbcData &);
    void _CleanUp();

    dynamics::FBModelPtr model_;

    std::vector<wbc::TaskPtr> task_list_;
    std::vector<wbc::ContactPtr> contact_list_;

    MatrixX A_;
    MatrixX Ainv_;
    VectorX grav_;
    VectorX coriolis_;

    VectorX full_config_;

    VectorX tau_ff_;
    VectorX des_jpos_;
    VectorX des_jvel_;

    std::array<double, robot::ModelAttrs::num_leg_joint> Kp_joint_, Kd_joint_;

    wbc::TaskPtr body_pos_task_;
    wbc::TaskPtr body_ori_task_;

    std::array<wbc::TaskPtr, robot::ModelAttrs::num_leg> foot_task_;
    std::array<wbc::ContactPtr, robot::ModelAttrs::num_leg> foot_contact_;

    std::array<Vector3, robot::ModelAttrs::num_leg> pre_foot_vel_;

    std::array<Vector3, robot::ModelAttrs::num_leg> Fr_result_;
    dynamics::Quat quat_des_;

    wbc::KinWbc kin_wbc_;
    wbc::Wbic wbic_;
  };

  using WbcPtr = std::shared_ptr<Wbc>;
}
