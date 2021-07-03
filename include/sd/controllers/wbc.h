#pragma once

#include <memory>
#include <array>
#include <vector>

#include "sd/estimators/state_est.h"
#include "sd/dynamics/fb_model.h"
#include "sd/controllers/leg.h"

namespace sd::ctrl
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

      void UpdateKp(size_t idx, double v) { _Kp(idx) = v; }

      const VectorXd &GetKp() { return _Kp; }

      void UpdateKd(size_t idx, double v) { _Kd(idx) = v; }

      const VectorXd &GetKd() { return _Kd; }

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

    protected:
      virtual bool _UpdateJc() = 0;
      virtual bool _UpdateJcDotQdot() = 0;
      virtual bool _UpdateUf() = 0;
      virtual bool _UpdateInequalityVector() = 0;
      size_t dim_contact_;
      size_t idx_Fz_;

      VectorXd Fr_des_;

      bool b_set_contact_ = false;
    };

    using ContactPtr = std::shared_ptr<Contact>;

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
  };

  using WbcPtr = std::shared_ptr<Wbc>;
}
