#pragma once

#include <memory>
#include <vector>

#include "common/eigen.h"
#include "wbc/contact.h"
#include "wbc/task.h"
#include "wbc/wbc.h"

namespace sdquadx::wbc {
class Wbic {
 public:
  using Ptr = std::unique_ptr<Wbic>;
  using Shared = std::shared_ptr<Wbic>;

  explicit Wbic(fpt_t weight);

  bool UpdateSetting(model::MassMatTp const &A, model::MassMatTp const &Ainv, model::GeneralFTp const &cori,
                     model::GeneralFTp const &grav);

  bool MakeTorque(SdVector12f &ret, std::vector<Task::ConstSharedPtr> const &task_list,
                  std::vector<Contact::ConstSharedPtr> const &contact_list);

 private:
  /* 为了方便阅读，拆分 MakeTorque 为多个私有方法。
   * 这些私有方法在接口上不是必须的，仅仅为了阅读。
   * 所以这些私有方法的参数使用了Eigen数据类型作为参数，并且没有使用Eigen::Ref，完全是为了方便。
   * 同样，为了定义这些私有方法，只能将 using Vector18
   * 提到这里的位置（本应该在cpp中）。
   */
  using Vector18 = Eigen::Matrix<fpt_t, consts::model::kDimConfig, 1>;

  bool _SetQPSize(MatrixX &G, VectorX &g0, MatrixX &CE, VectorX &ce0, MatrixX &CI, VectorX &ci0, MatrixX &Uf,
                  VectorX &Uf_ieq_vec, MatrixX &Jc, VectorX &JcDotQdot, VectorX &Fr_des,
                  std::vector<Contact::ConstSharedPtr> const &contact_list);

  bool _ContactBuilding(MatrixX &Uf, VectorX &Uf_ieq_vec,
                        MatrixX &Jc,  //  , num_qdot_
                        VectorX &JcDotQdot, VectorX &Fr_des, std::vector<Contact::ConstSharedPtr> const &contact_list);
  bool _SetEqualityConstraint(MatrixX &CE, VectorX &ce0, MatrixX const &Jc, VectorX const &Fr_des,
                              Vector18 const &qddot);
  bool _SetInEqualityConstraint(MatrixX &CI, VectorX &ci0, MatrixX const &Uf, VectorX const &Uf_ieq_vec,
                                VectorX const &Fr_des);

  bool _SetCost(MatrixX &G);

  bool _GetSolution(SdVector12f &ret, Vector18 const &qddot, VectorX const &z, VectorX const &Fr_des,
                    MatrixX const &Jc);

  bool _WeightedInverse(MatrixX &ret, MatrixX const &J, MatrixX const &Winv, fpt_t threshold = 0.0001);

  std::array<fpt_t, 6 *consts::model::kDimConfig> Sv_ = {};  // Virtual joint
  model::MassMatTp A_;
  model::MassMatTp Ainv_;
  model::GeneralFTp cori_;
  model::GeneralFTp grav_;

  bool b_updatesetting_ = false;

  int dim_opt_;
  int dim_eq_cstr_;  // equality constraints

  int dim_rf_;
  int dim_Uf_;

  // Input
  SdVector6f W_floating_;
  SdVector18f W_rf_;
};
}  // namespace sdquadx::wbc
