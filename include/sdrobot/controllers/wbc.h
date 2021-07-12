#pragma once

#include <memory>
#include <array>
#include <vector>

#include "sdrobot/estimators/state_est.h"
#include "sdrobot/dynamics/fb_model.h"
#include "sdrobot/controllers/leg.h"

#include "QuadProgpp/QuadProg++.hh"

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

      bool UpdateTask(const VectorXd &pos_des, const VectorXd &vel_des,
                      const VectorXd &acc_des)
      {
        _UpdateTaskJacobian();
        _UpdateTaskJDotQdot();
        _UpdateCommand(pos_des, vel_des, acc_des);
        _AdditionalUpdate();
        b_set_task_ = true;
        return true;
      }

      const VectorXd &getCommand() { return op_cmd_; }
      const MatrixXd &getTaskJacobian() { return Jt_; }
      const VectorXd &getTaskJacobianDotQdot() { return JtDotQdot_; }

      void UpdateKp(size_t idx, double v) { _Kp(idx) = v; }

      const VectorXd &GetKp() { return _Kp; }

      void UpdateKd(size_t idx, double v) { _Kd(idx) = v; }

      const VectorXd &GetKd() { return _Kd; }

      const VectorXd &getPosError() { return pos_err_; }
      const VectorXd &getDesVel() { return vel_des_; }
      const VectorXd &getDesAcc() { return acc_des_; }

    protected:
      // Update op_cmd_
      virtual bool _UpdateCommand(const VectorXd &pos_des, const VectorXd &vel_des,
                                  const VectorXd &acc_des) = 0;
      // Update Jt_
      virtual bool _UpdateTaskJacobian() = 0;
      // Update JtDotQdot_
      virtual bool _UpdateTaskJDotQdot() = 0;
      // Additional Update (defined in child classes)
      virtual bool _AdditionalUpdate() = 0;

      bool b_set_task_ = false;
      size_t dim_task_;

      const dynamics::FBModelPtr _robot_sys;

      VectorXd JtDotQdot_;
      MatrixXd Jt_;

      VectorXd op_cmd_;
      VectorXd pos_err_;
      VectorXd vel_des_, acc_des_;

      VectorXd _Kp, _Kd;
    };

    using TaskPtr = std::shared_ptr<Task>;

    class Contact
    {
    public:
      Contact(size_t dim) : dim_contact_(dim)
      {
        idx_Fz_ = dim - 1; // because normally (tau_x,y,z , linear_x,y,z)
        Fr_des_ = VectorXd::Zero(dim);
      }

      size_t getDim() const { return dim_contact_; }
      size_t getDimRFConstraint() const { return Uf_.rows(); }
      size_t getFzIndex() const { return idx_Fz_; }

      const VectorXd &getRFDesired() { return Fr_des_; }
      void setRFDesired(const VectorXd &Fr_des) { Fr_des_ = Fr_des; }
      bool UpdateContact()
      {
        _UpdateJc();
        _UpdateJcDotQdot();
        _UpdateUf();
        _UpdateInequalityVector();
        b_set_contact_ = true;
        return true;
      }
      const MatrixXd &getContactJacobian() { return Jc_; }
      const VectorXd &getJcDotQdot() { return JcDotQdot_; }
      const MatrixXd &getRFConstraintMtx() { return Uf_; }
      const VectorXd &getRFConstraintVec() { return ieq_vec_; }

    protected:
      virtual bool _UpdateJc() = 0;
      virtual bool _UpdateJcDotQdot() = 0;
      virtual bool _UpdateUf() = 0;
      virtual bool _UpdateInequalityVector() = 0;

      size_t dim_contact_;
      size_t idx_Fz_;
      MatrixXd Uf_;
      VectorXd ieq_vec_;

      VectorXd Fr_des_;

      MatrixXd Jc_;
      VectorXd JcDotQdot_;

      bool b_set_contact_ = false;
    };

    using ContactPtr = std::shared_ptr<Contact>;

    class Wbic
    {
    public:
      Wbic(size_t num_qdot, double weight);

      void UpdateSetting(const MatrixXd &A, const MatrixXd &Ainv,
                         const VectorXd &cori, const VectorXd &grav);

      void MakeTorque(VectorXd &cmd, const std::vector<TaskPtr> &task_list, const std::vector<ContactPtr> &contact_list);

    private:
      void _ContactBuilding(const std::vector<ContactPtr> &contact_list);
      void _SetEqualityConstraint(const VectorXd &qddot);
      void _SetInEqualityConstraint();

      void _SetOptimizationSize(const std::vector<ContactPtr> &contact_list);
      void _SetCost();
      void _GetSolution(const VectorXd &qddot, VectorXd &cmd);

      void _WeightedInverse(const MatrixXd &J, const MatrixXd &Winv, MatrixXd &Jinv,
                            double threshold = 0.0001);

      size_t num_act_joint_;
      size_t num_qdot_;

      MatrixXd Sa_; // Actuated joint
      MatrixXd Sv_; // Virtual joint

      MatrixXd A_;
      MatrixXd Ainv_;
      VectorXd cori_;
      VectorXd grav_;

      bool b_updatesetting_ = false;

      size_t _dim_opt;
      size_t _dim_eq_cstr; // equality constraints

      size_t _dim_rf;
      size_t _dim_Uf;

      size_t _dim_floating;

      quadprogpp::Vector<double> z;
      // Cost
      quadprogpp::Matrix<double> G;
      quadprogpp::Vector<double> g0;

      // Equality
      quadprogpp::Matrix<double> CE;
      quadprogpp::Vector<double> ce0;

      // Inequality
      quadprogpp::Matrix<double> CI;
      quadprogpp::Vector<double> ci0;

      MatrixXd _dyn_CE;
      VectorXd _dyn_ce0;
      MatrixXd _dyn_CI;
      VectorXd _dyn_ci0;

      MatrixXd _eye;

      MatrixXd _Uf;
      VectorXd _Uf_ieq_vec;

      MatrixXd _Jc;
      VectorXd _JcDotQdot;
      VectorXd _Fr_des;

      // Extra data
      // Output
      VectorXd _opt_result;
      VectorXd _qddot;
      VectorXd _Fr;

      // Input
      VectorXd _W_floating;
      VectorXd _W_rf;
    };

    class KinWbc
    {
    public:
      KinWbc(size_t num_qdot);
      bool FindConfiguration(const VectorXd &curr_config,
                             const std::vector<TaskPtr> &task_list, const std::vector<ContactPtr> &contact_list,
                             VectorXd &jpos_cmd, VectorXd &jvel_cmd);

    private:
      void _PseudoInverse(const MatrixXd &J, MatrixXd &Jinv);
      void _BuildProjectionMatrix(const MatrixXd &J, MatrixXd &N);

      double threshold_ = 0.001;
      size_t num_qdot_;
      size_t num_act_joint_;
      MatrixXd I_mtx;
    };

  } // namespace wbc

  struct WbcData
  {
    Vector3d pBody_des;
    Vector3d vBody_des;
    Vector3d aBody_des;
    Vector3d pBody_RPY_des;
    Vector3d vBody_Ori_des;

    std::array<Vector3d, 4> pFoot_des;
    std::array<Vector3d, 4> vFoot_des;
    std::array<Vector3d, 4> aFoot_des;
    std::array<Vector3d, 4> Fr_des;

    Vector4d contact_state;
  };

  class Wbc
  {
  public:
    Wbc(const dynamics::FBModelPtr &model, double weight);
    void Run(const WbcData &, const est::StateEstPtr &, LegPtr &);

  private:
    void _UpdateModel(const est::StateData &, const robot::leg::Datas &);
    void _ComputeWBC();
    void _UpdateLegCMD(LegPtr &);
    void _ContactTaskUpdate(const WbcData &, LegPtr &);
    void _CleanUp();

    dynamics::FBModelPtr model_;

    // TODO rename vars
    std::vector<wbc::TaskPtr> _task_list;
    std::vector<wbc::ContactPtr> _contact_list;

    MatrixXd _A;
    MatrixXd _Ainv;
    VectorXd _grav;
    VectorXd _coriolis;

    VectorXd _full_config;

    VectorXd _tau_ff;
    VectorXd _des_jpos;
    VectorXd _des_jvel;

    std::array<double, robot::ModelAttrs::num_leg_joint> _Kp_joint, _Kd_joint;

    wbc::TaskPtr _body_pos_task;
    wbc::TaskPtr _body_ori_task;

    std::array<wbc::TaskPtr, robot::ModelAttrs::num_leg> _foot_task;
    std::array<wbc::ContactPtr, robot::ModelAttrs::num_leg> _foot_contact;

    std::array<Vector3d, robot::ModelAttrs::num_leg> pre_foot_vel;

    std::array<Vector3d, robot::ModelAttrs::num_leg> _Fr_result;
    dynamics::Quat _quat_des;

    wbc::KinWbc _kin_wbc;
    wbc::Wbic _wbic;
  };

  using WbcPtr = std::shared_ptr<Wbc>;
}
