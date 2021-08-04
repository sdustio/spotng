#pragma once

#include "wbc/in_data.h"
#include "wbc/task.h"
#include "wbc/contact.h"

namespace sdrobot::wbc
{
  using TorqueTp = std::array<fpt_t, params::model::num_act_joint>;

  class Wbic
  {
  public:
    using Ptr = std::unique_ptr<Wbic>;
    using Shared = std::shared_ptr<Wbic>;

    Wbic(fpt_t weight);

    bool UpdateSetting(model::MassMatTp const &A, model::MassMatTp const &Ainv,
                       model::GeneralFTp const &cori, model::GeneralFTp const &grav);

    bool MakeTorque(TorqueTp &ret, const std::vector<Task::Ptr> &task_list, const std::vector<Contact::Ptr> &contact_list);

  private:
    bool _SetQPSize(MatrixX &G,
                    VectorX &g0,
                    MatrixX &CE,
                    VectorX &ce0,
                    MatrixX &CI,
                    VectorX &ci0,
                    MatrixX &Uf,
                    VectorX &Uf_ieq_vec,
                    MatrixX &Jc,
                    VectorX &JcDotQdot,
                    VectorX &Fr_des,
                    std::vector<Contact::Ptr> const &contact_list);

    bool _ContactBuilding(MatrixX &Uf,
                          VectorX &Uf_ieq_vec,
                          MatrixX &Jc, //  , num_qdot_
                          VectorX &JcDotQdot,
                          VectorX &Fr_des,
                          std::vector<Contact::Ptr> const &contact_list);
    bool _SetEqualityConstraint(MatrixX &CE, VectorX &ce0,
                                MatrixX const &Jc,
                                VectorX const &Fr_des,
                                Vector18 const &qddot);
    bool _SetInEqualityConstraint(MatrixX &CI, VectorX &ci0,
                                  MatrixX const &Uf,
                                  VectorX const &Uf_ieq_vec,
                                  VectorX const &Fr_des);

    bool _SetCost(MatrixX &G);

    bool _GetSolution(TorqueTp &ret,
                      Vector18 const &qddot,
                      VectorX const &z,
                      VectorX const &Fr_des,
                      MatrixX const &Jc);

    bool _WeightedInverse(MatrixX &ret,
                          MatrixX const &J,
                          MatrixX const &Winv,
                          fpt_t threshold = 0.0001);

    std::array<fpt_t, 6 *params::model::dim_config> Sv_ = {}; // Virtual joint
    std::array<fpt_t, params::model::dim_config * params::model::dim_config> eye_;

    model::MassMatTp A_;
    model::MassMatTp Ainv_;
    model::GeneralFTp cori_;
    model::GeneralFTp grav_;

    bool b_updatesetting_ = false;

    int dim_opt_;
    int dim_eq_cstr_; // equality constraints

    int dim_rf_;
    int dim_Uf_;

    // Input
    SdVector6f W_floating_;
    TorqueTp W_rf_;
  };
} // namespace sdrobot::wbc
