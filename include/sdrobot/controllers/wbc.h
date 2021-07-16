#pragma once

#include <memory>
#include <array>
#include <vector>

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
      Task(size_t dim, const dynamics::FBModelPtr &model)
          : dim_task_(dim),
            _robot_sys(model),
            op_cmd_(dim),
            pos_err_(dim),
            vel_des_(dim),
            acc_des_(dim),
            _Kp(dim),
            _Kd(dim) {}

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

      const VectorX &getCommand() { return op_cmd_; }
      const MatrixX &getTaskJacobian() { return Jt_; }
      const VectorX &getTaskJacobianDotQdot() { return JtDotQdot_; }

      void UpdateKp(size_t idx, double v) { _Kp(idx) = v; }

      const VectorX &GetKp() { return _Kp; }

      void UpdateKd(size_t idx, double v) { _Kd(idx) = v; }

      const VectorX &GetKd() { return _Kd; }

      const VectorX &getPosError() { return pos_err_; }
      const VectorX &getDesVel() { return vel_des_; }
      const VectorX &getDesAcc() { return acc_des_; }

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
      size_t dim_task_;

      const dynamics::FBModelPtr _robot_sys;

      VectorX JtDotQdot_;
      MatrixX Jt_;

      VectorX op_cmd_;
      VectorX pos_err_;
      VectorX vel_des_, acc_des_;

      VectorX _Kp_kin;
      VectorX _Kp, _Kd;
    };

    using TaskPtr = std::shared_ptr<Task>;

    class Contact
    {
    public:
      Contact(size_t dim) : dim_contact_(dim)
      {
        idx_Fz_ = dim - 1; // because normally (tau_x,y,z , linear_x,y,z)
        Fr_des_ = VectorX::Zero(dim);
      }

      size_t getDim() const { return dim_contact_; }
      size_t getDimRFConstraint() const { return Uf_.rows(); }
      size_t getFzIndex() const { return idx_Fz_; }

      const VectorX &getRFDesired() { return Fr_des_; }
      void setRFDesired(const VectorX &Fr_des) { Fr_des_ = Fr_des; }
      bool UpdateContact()
      {
        _UpdateJc();
        _UpdateJcDotQdot();
        _UpdateUf();
        _UpdateInequalityVector();
        b_set_contact_ = true;
        return true;
      }
      const MatrixX &getContactJacobian() { return Jc_; }
      const VectorX &getJcDotQdot() { return JcDotQdot_; }
      const MatrixX &getRFConstraintMtx() { return Uf_; }
      const VectorX &getRFConstraintVec() { return ieq_vec_; }

    protected:
      virtual bool _UpdateJc() = 0;
      virtual bool _UpdateJcDotQdot() = 0;
      virtual bool _UpdateUf() = 0;
      virtual bool _UpdateInequalityVector() = 0;

      size_t idx_Fz_;
      MatrixX Uf_;
      VectorX ieq_vec_;

      VectorX Fr_des_;

      MatrixX Jc_;
      VectorX JcDotQdot_;

      size_t dim_contact_;
      bool b_set_contact_ = false;
    };

    using ContactPtr = std::shared_ptr<Contact>;

    class Wbic
    {
    public:
      Wbic(size_t num_qdot, double weight);

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

      size_t num_act_joint_;
      size_t num_qdot_;

      MatrixX Sa_; // Actuated joint
      MatrixX Sv_; // Virtual joint

      MatrixX A_;
      MatrixX Ainv_;
      VectorX cori_;
      VectorX grav_;

      bool b_updatesetting_ = false;

      size_t _dim_opt;
      size_t _dim_eq_cstr; // equality constraints

      size_t _dim_rf;
      size_t _dim_Uf;

      size_t _dim_floating = 6;

      VectorX z;
      // Cost
      MatrixX G;
      VectorX g0;

      // Equality
      MatrixX _dyn_CE;
      VectorX _dyn_ce0;

      // Inequality
      MatrixX _dyn_CI;
      VectorX _dyn_ci0;

      MatrixX _eye;

      MatrixX _Uf;
      VectorX _Uf_ieq_vec;

      MatrixX _Jc;
      VectorX _JcDotQdot;
      VectorX _Fr_des;

      // Extra data
      // Output
      VectorX _opt_result;
      VectorX _qddot;
      VectorX _Fr;

      // Input
      VectorX _W_floating;
      VectorX _W_rf;
    };

    class KinWbc
    {
    public:
      KinWbc(size_t num_qdot);
      bool FindConfiguration(const VectorX &curr_config,
                             const std::vector<TaskPtr> &task_list, const std::vector<ContactPtr> &contact_list,
                             VectorX &jpos_cmd, VectorX &jvel_cmd);

    private:
      void _PseudoInverse(const MatrixX &J, MatrixX &Jinv);
      void _BuildProjectionMatrix(const MatrixX &J, MatrixX &N);

      double threshold_ = 0.001;
      size_t num_qdot_;
      size_t num_act_joint_;
      MatrixX I_mtx;
    };

  } // namespace wbc

  struct WbcData
  {
    Vector3 pBody_des;
    Vector3 vBody_des;
    Vector3 aBody_des;
    Vector3 pBody_RPY_des;
    Vector3 vBody_Ori_des;

    std::array<Vector3, 4> pFoot_des;
    std::array<Vector3, 4> vFoot_des;
    std::array<Vector3, 4> aFoot_des;

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

    // TODO rename vars
    std::vector<wbc::TaskPtr> _task_list;
    std::vector<wbc::ContactPtr> _contact_list;

    MatrixX _A;
    MatrixX _Ainv;
    VectorX _grav;
    VectorX _coriolis;

    VectorX _full_config;

    VectorX _tau_ff;
    VectorX _des_jpos;
    VectorX _des_jvel;

    std::array<double, robot::ModelAttrs::num_leg_joint> _Kp_joint, _Kd_joint;

    wbc::TaskPtr _body_pos_task;
    wbc::TaskPtr _body_ori_task;

    std::array<wbc::TaskPtr, robot::ModelAttrs::num_leg> _foot_task;
    std::array<wbc::ContactPtr, robot::ModelAttrs::num_leg> _foot_contact;

    std::array<Vector3, robot::ModelAttrs::num_leg> pre_foot_vel;

    std::array<Vector3, robot::ModelAttrs::num_leg> _Fr_result;
    dynamics::Quat _quat_des;

    wbc::KinWbc _kin_wbc;
    wbc::Wbic _wbic;
  };

  using WbcPtr = std::shared_ptr<Wbc>;
}
