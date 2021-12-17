#include "estimate/pos_vel.h"

#include "sdquadx/consts.h"

namespace sdquadx::estimate {

namespace params {
constexpr inline fpt_t const kBigNumber = 100;
}  // namespace params

PosVel::PosVel(Options::ConstSharedPtr const &opts) : opts_(opts) {
  xhat_.fill(0.);
  ps_.fill(0.);
  vs_.fill(0.);
  // 状态转移矩阵，计算K+1时刻状态值X[k+1] 自己写出来算下就知道
  Eigen::Map<Matrix18> A(A_.data());
  A.setZero();
  A.block<3, 3>(0, 0) = Matrix3::Identity();
  A.block<3, 3>(0, 3) = opts_->ctrl_sec * Matrix3::Identity();
  A.block<3, 3>(3, 3) = Matrix3::Identity();
  A.block<12, 12>(6, 6) = Eigen::Matrix<fpt_t, 12, 12>::Identity();
  // 输入矩阵
  Eigen::Map<Eigen::Matrix<fpt_t, 18, 3>> B(B_.data());
  B.setZero();
  B.block<3, 3>(3, 0) = opts_->ctrl_sec * Matrix3::Identity();
  // 观测矩阵
  // 观测量[p1 p2 p3 p4 v1 v2 v3 v4 z1 z2 z3 z4]（v
  // z都在世界坐标下p在机身坐标系） p v 是向量 z是标量 pib=pb-piw vi=vb
  Eigen::Matrix<fpt_t, 3, 6> C1;
  C1 << Matrix3::Identity(), Matrix3::Zero();
  Eigen::Matrix<fpt_t, 3, 6> C2;
  C2 << Matrix3::Zero(), Matrix3::Identity();
  Eigen::Map<Eigen::Matrix<fpt_t, 28, 18>> C(C_.data());
  C.setZero();
  C.block<3, 6>(0, 0) = C1;
  C.block<3, 6>(3, 0) = C1;
  C.block<3, 6>(6, 0) = C1;
  C.block<3, 6>(9, 0) = C1;
  C.block<12, 12>(0, 6) = -1. * Eigen::Matrix<fpt_t, 12, 12>::Identity();
  C.block<3, 6>(12, 0) = C2;
  C.block<3, 6>(15, 0) = C2;
  C.block<3, 6>(18, 0) = C2;
  C.block<3, 6>(21, 0) = C2;
  C(27, 17) = 1.;
  C(26, 14) = 1.;
  C(25, 11) = 1.;
  C(24, 8) = 1.;

  // 初始不确定性
  Eigen::Map<Matrix18> P(P_.data());
  P.setIdentity();
  P = 100. * P;
  // 初始状态估计噪声
  Eigen::Map<Matrix18> Q0(Q0_.data());
  Q0.setIdentity();
  Q0.block<3, 3>(0, 0) = (opts_->ctrl_sec / 20.) * opts_->estimate.process_noise_pimu * Matrix3::Identity();
  Q0.block<3, 3>(3, 3) =
      (opts_->ctrl_sec * opts_->gravity / 20.) * opts_->estimate.process_noise_vimu * Matrix3::Identity();
  Q0.block<12, 12>(6, 6) =
      opts_->ctrl_sec * opts_->estimate.process_noise_pfoot * Eigen::Matrix<fpt_t, 12, 12>::Identity();

  Eigen::Map<Matrix28> R0(R0_.data());
  R0.setIdentity();
  R0.block<12, 12>(0, 0) *= opts_->estimate.sensor_noise_pfoot;
  R0.block<12, 12>(12, 12) *= opts_->estimate.sensor_noise_vfoot;
  R0.block<4, 4>(24, 24) *= opts_->estimate.sensor_noise_zfoot;
}

bool PosVel::RunOnce(State &ret) {
  if (!ret.success) return false;

  Eigen::Map<Vector18> xhat(xhat_.data());
  Eigen::Map<Vector12> ps(ps_.data());
  Eigen::Map<Vector12> vs(vs_.data());
  Eigen::Map<Matrix18> A(A_.data());
  Eigen::Map<Eigen::Matrix<fpt_t, 18, 3>> B(B_.data());
  Eigen::Map<Eigen::Matrix<fpt_t, 28, 18>> C(C_.data());
  Eigen::Map<Matrix18> P(P_.data());
  Eigen::Map<Matrix18> Q0(Q0_.data());
  Eigen::Map<Matrix28> R0(R0_.data());
  Eigen::Map<Matrix18> Q(Q_.data());
  Eigen::Map<Matrix28> R(R_.data());

  auto rot_mat = ToConstEigenTp(ret.rot_mat);
  // 状态估计噪声
  Q = Q0;

  // 观测噪声矩阵
  R = R0;

  int qindex = 0;
  int rindex1 = 0;
  int rindex2 = 0;
  int rindex3 = 0;
  // 重力向量
  Matrix3 Rbod = rot_mat.transpose();  // 机身到世界的变换矩阵
  // 输入量a 世界下
  Vector3 acc = ToConstEigenTp(ret.acc) + Vector3(0, 0, -opts_->gravity);

  // std::cout << "A WORLD\n" << acc << "\n";
  Vector4 pzs = Vector4::Zero();
  Vector3 p0, v0;
  // 初始位置 速度
  p0 << xhat_[0], xhat_[1], xhat_[2];
  v0 << xhat_[3], xhat_[4], xhat_[5];

  // 构成状态变量等
  for (int i = 0; i < consts::model::kNumLeg; i++) {
    CalcFootPosVelRobot(ret.foot_pos_robot[i], ret.foot_vel_robot[i], i, ret);
    auto p_rel = ToConstEigenTp(ret.foot_pos_robot[i]);
    auto pd_rel = ToConstEigenTp(ret.foot_vel_robot[i]);
    auto p_f = ToEigenTp(ret.foot_pos[i]);
    auto pd_f = ToEigenTp(ret.foot_vel[i]);

    p_f = Rbod * p_rel;  // 足端位置在世界坐标系描述 即方向 大小 没有位置
    pd_f = Rbod * (ToConstEigenTp(ret.avel_robot).cross(p_rel) + pd_rel);

    // 更新四条腿用索引
    int i1 = 3 * i;
    qindex = 6 + i1;
    rindex1 = i1;       // p
    rindex2 = 12 + i1;  // V
    rindex3 = 24 + i;   // Z

    fpt_t trust = 1.;  // 支撑: 1，摆动: 0

    // 获取接触状态 在整个支撑过程百分比(从0到1)  完成后为0
    fpt_t phase = std::fmin(ret.contact[i], 1.);

    // 脚i的测量协方差在摆动过程中被提高到一个很高的值，因此在这个融合过程中，摆动腿的测量值被有效地忽略了
    fpt_t trust_window = 0.2;

    // trust 一般为1
    // 在开始和结束支撑相窗口范围 大概率为摆动 trust -> 0
    if (phase < trust_window) {
      trust = phase / trust_window;
    } else if (phase > (1. - trust_window)) {
      trust = (1. - phase) / trust_window;
    }
    // 摆动腿和支撑腿刚触地，即将离地时状态，测量噪声协方差增大
    Q.block<3, 3>(qindex, qindex) = (1. + (1. - trust) * params::kBigNumber) * Q.block<3, 3>(qindex, qindex);  // p

    R.block<3, 3>(rindex1, rindex1) = 1 * R.block<3, 3>(rindex1, rindex1);                                         // p
    R.block<3, 3>(rindex2, rindex2) = (1. + (1. - trust) * params::kBigNumber) * R.block<3, 3>(rindex2, rindex2);  // v
    R(rindex3, rindex3) = (1. + (1. - trust) * params::kBigNumber) * R(rindex3, rindex3);                          // z

    // 处理后的
    ps.segment(i1, 3) = -p_f;
    vs.segment(i1, 3) = (1. - trust) * v0 + trust * (-pd_f);
    pzs(i) = (1.0 - trust) * (p0(2) + p_f(2));
  }

  Vector28 y;

  y << ps, vs, pzs;

  // 卡尔曼滤波
  xhat = A * xhat + B * acc;  // 状态预测方程

  P = A * P * A.transpose() + Q;  // 不确定性预测方程

  // 卡尔曼增益准备
  MatrixX S = C * P * C.transpose() + R;

  Vector28 yModel = C * xhat;  // 预测的观测值

  Vector28 ey = y - yModel;  // 误差 卡尔曼增益计算准备

  Vector28 S_ey = S.lu().solve(ey);  // 求逆

  xhat += P * C.transpose() * S_ey;

  MatrixX S_C = S.lu().solve(C);  // 求逆

  P = (Matrix18::Identity() - P * C.transpose() * S_C) * P;

  // 修正 P
  P = (P + P.transpose()) / 2.;
  if (P.block<2, 2>(0, 0).determinant() > 0.000001) {
    P.block<2, 16>(0, 2).setZero();
    P.block<16, 2>(2, 0).setZero();
    P.block<2, 2>(0, 0) /= 10.;
  }
  // 输出状态量
  auto pos = ToEigenTp(ret.pos);
  pos = xhat.block<3, 1>(0, 0);
  auto lvel = ToEigenTp(ret.lvel);
  lvel = xhat.block<3, 1>(3, 0);
  ToEigenTp(ret.lvel_robot) = rot_mat * ToConstEigenTp(ret.lvel);

  // foot world pos and vel
  for (int i = 0; i < consts::model::kNumLeg; i++) {
    ToEigenTp(ret.foot_pos[i]) = pos + ToConstEigenTp(ret.foot_pos[i]);
    ToEigenTp(ret.foot_vel[i]) = lvel + ToConstEigenTp(ret.foot_vel[i]);
  }

  return true;
}

bool PosVel::CalcFootPosVelRobot(SdVector3f &pos, SdVector3f &vel, int leg, State const &data) {
  fpt_t l1 = opts_->model.link_length_abad;
  fpt_t l2 = opts_->model.link_length_hip;
  fpt_t l3 = opts_->model.link_length_knee;
  fpt_t hx = opts_->model.location_abad_fl[0];
  fpt_t hy = opts_->model.location_abad_fl[1];
  fpt_t sign_fh = consts::model::kSignFH[leg];
  fpt_t sign_lr = consts::model::kSignLR[leg];

  auto const &q = data.q[leg];
  fpt_t s1 = std::sin(q[0]);
  fpt_t s2 = std::sin(q[1]);
  fpt_t s3 = std::sin(q[2]);

  fpt_t c1 = std::cos(q[0]);
  fpt_t c2 = std::cos(q[1]);
  fpt_t c3 = std::cos(q[2]);

  fpt_t c23 = c2 * c3 - s2 * s3;
  fpt_t s23 = s2 * c3 + c2 * s3;

  // [DIFF] pos[0], J(0, :) 符号相反
  pos[0] = -l3 * s23 - l2 * s2 + sign_fh * hx;
  pos[1] = l1 * sign_lr * c1 + l3 * (s1 * c23) + l2 * c2 * s1 + sign_lr * hy;
  pos[2] = l1 * sign_lr * s1 - l3 * (c1 * c23) - l2 * c1 * c2;

  Matrix3 J;
  J(0, 0) = 0;
  J(0, 1) = -l3 * c23 - l2 * c2;
  J(0, 2) = -l3 * c23;
  J(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - l1 * sign_lr * s1;
  J(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
  J(1, 2) = -l3 * s1 * s23;
  J(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + l1 * sign_lr * c1;
  J(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
  J(2, 2) = l3 * c1 * s23;

  ToEigenTp(vel) = J * ToConstEigenTp(data.qd[leg]);
  return true;
}
}  // namespace sdquadx::estimate
