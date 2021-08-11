#include "estimate/pos_vel.h"
#include "sdrobot/params.h"

namespace sdrobot::estimate
{
  PosVel::PosVel(
      fpt_t dt,
      fpt_t gravity,
      leg::LegCtrl::ConstSharedPtr const &legctrl,
      model::Quadruped::ConstSharedPtr const &quad) : dt_(dt),
                                                 gravity_(gravity),
                                                 legctrl_(legctrl),
                                                 quad_(quad)
  {
    xhat_.fill(0.);
    ps_.fill(0.);
    vs_.fill(0.);
    //状态转移矩阵，计算K+1时刻状态值X[k+1] 自己写出来算下就知道
    Eigen::Map<Matrix18> A(A_.data());
    A.setZero();
    A.block<3, 3>(0, 0) = Matrix3::Identity();
    A.block<3, 3>(0, 3) = dt_ * Matrix3::Identity();
    A.block<3, 3>(3, 3) = Matrix3::Identity();
    A.block<12, 12>(6, 6) = Eigen::Matrix<fpt_t, 12, 12>::Identity();
    //输入矩阵
    Eigen::Map<Eigen::Matrix<fpt_t, 18, 3>> B(B_.data());
    B.setZero();
    B.block<3, 3>(3, 0) = dt_ * Matrix3::Identity();
    //观测矩阵
    //观测量[p1 p2 p3 p4 v1 v2 v3 v4 z1 z2 z3 z4]（v z都在世界坐标下p在机身坐标系） p v 是向量 z是标量 pib=pb-piw vi=vb
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

    //初始不确定性
    Eigen::Map<Matrix18> P(P_.data());
    P.setIdentity();
    P = 100. * P;
    //初始状态估计噪声
    Eigen::Map<Matrix18> Q0(Q0_.data());
    Q0.setIdentity();
    Q0.block<3, 3>(0, 0) = (dt_ / 20.) * Matrix3::Identity();
    Q0.block<3, 3>(3, 3) =
        (dt_ * gravity_ / 20.) * Matrix3::Identity();

    Q0.block<12, 12>(6, 6) = dt_ * Eigen::Matrix<fpt_t, 12, 12>::Identity();

    Eigen::Map<Matrix28> R0(R0_.data());
    R0.setIdentity();
  }

  bool PosVel::RunOnce(State &ret)
  {

    fpt_t process_noise_pimu = params::noise::kIMUProcessNoisePosition;
    fpt_t process_noise_vimu = params::noise::kIMUProcessNoiseVelocity;
    fpt_t process_noise_pfoot = params::noise::kFootProcessNoisePosition;
    fpt_t sensor_noise_pimu_rel_foot = params::noise::kFootSensorNoisePosition;
    fpt_t sensor_noise_vimu_rel_foot = params::noise::kFootSensorNoiseVelocity;
    fpt_t sensor_noise_zfoot = params::noise::kFootHeightSensorNoise;

    Eigen::Map<Vector18> xhat(xhat_.data());
    Eigen::Map<Vector12> ps(ps_.data());
    Eigen::Map<Vector12> vs(vs_.data());
    Eigen::Map<Matrix18> A(A_.data());
    Eigen::Map<Eigen::Matrix<fpt_t, 18, 3>> B(B_.data());
    Eigen::Map<Eigen::Matrix<fpt_t, 28, 18>> C(C_.data());
    Eigen::Map<Matrix18> P(P_.data());
    Eigen::Map<Matrix18> Q0(Q0_.data());
    Eigen::Map<Matrix28> R0(R0_.data());

    auto rot_mat = ToConstEigenTp(ret.rot_mat);
    auto const &datas = legctrl_->GetDatas();

    //状态估计噪声
    MatrixX Q = Matrix18::Identity();;
    Q.block<3, 3>(0, 0) = Q0.block<3, 3>(0, 0) * process_noise_pimu;
    Q.block<3, 3>(3, 3) = Q0.block<3, 3>(3, 3) * process_noise_vimu;
    Q.block<12, 12>(6, 6) = Q0.block<12, 12>(6, 6) * process_noise_pfoot;
    //观测噪声矩阵
    MatrixX R = Matrix28::Identity();
    R.block<12, 12>(0, 0) = R0.block<12, 12>(0, 0) * sensor_noise_pimu_rel_foot;
    R.block<12, 12>(12, 12) =
        R0.block<12, 12>(12, 12) * sensor_noise_vimu_rel_foot;
    R.block<4, 4>(24, 24) = R0.block<4, 4>(24, 24) * sensor_noise_zfoot;

    int qindex = 0;
    int rindex1 = 0;
    int rindex2 = 0;
    int rindex3 = 0;
    //重力向量
    Vector3 g(0, 0, -gravity_);
    Matrix3 Rbod = rot_mat.transpose(); //机身到世界的变换矩阵
    // in old code, Rbod * se_acc + g
    //输入量a 世界下
    Vector3 acc = ToConstEigenTp(ret.acc) + g;

    // std::cout << "A WORLD\n" << acc << "\n";
    Vector4 pzs = Vector4::Zero();
    Vector4 trusts = Vector4::Zero();
    Vector3 p0, v0;
    //初始位置 速度
    p0 << xhat_[0], xhat_[1], xhat_[2];
    v0 << xhat_[3], xhat_[4], xhat_[5];

    //构成状态变量等
    for (int i = 0; i < params::model::kNumLeg; i++)
    {
      int i1 = 3 * i;
      SdVector3f _ph;
      quad_->CalcHipLocation(_ph, i); // hip positions relative to CoM 相对于CoM的髋位置
      Eigen::Map<Vector3> ph(_ph.data());

      Eigen::Map<Vector3 const> datap(datas[i].p.data());
      Vector3 p_rel = ph + datap; //足端位置在机身坐标系

      Eigen::Map<Vector3 const> datav(datas[i].v.data());
      Vector3 dp_rel = datav;     //足端速度在机身坐标系
      Vector3 p_f = Rbod * p_rel; //足端位置在世界坐标系描述 即方向 大小 没有位置

      //足端速度在世界坐标系描述 机身转动导致足端速度+足端本身速度
      Vector3 dp_f =
          Rbod * (ToConstEigenTp(ret.avel_robot).cross(p_rel) + dp_rel);

      //更新四条腿用索引
      qindex = 6 + i1;
      rindex1 = i1;      //p
      rindex2 = 12 + i1; //V
      rindex3 = 24 + i;  //Z

      fpt_t trust = 1.;
      fpt_t phase = fmin(ret.contact[i], 1.); //获得接触状态估计
                                               //获取接触状态 在整个支撑过程百分比(从0到1)  完成后为0
                                               //脚i的测量协方差在摆动过程中被提高到一个很高的值，因此在这个融合过程中，摆动腿的测量值被有效地忽略了
      //fptype trust_window = fptype(0.25);
      fpt_t trust_window = 0.2;

      //在开始和结束支撑相窗口范围 当前相位在窗口范围的百分比
      //trust 一般为1
      if (phase < trust_window)
      {
        trust = phase / trust_window;
      }
      else if (phase > (1. - trust_window))
      {
        trust = (1. - phase) / trust_window;
      }
      //fptype high_suspect_number(1000);
      fpt_t high_suspect_number = 100.;

      // printf("Trust %d: %.3f\n", i, trust);
      //摆动腿和支撑腿刚触地，即将离地时状态，测量噪声协方差增大
      Q.block<3, 3>(qindex, qindex) =
          (1. + (1. - trust) * high_suspect_number) * Q.block<3, 3>(qindex, qindex); //p

      R.block<3, 3>(rindex1, rindex1) = 1 * R.block<3, 3>(rindex1, rindex1);                                         //p
      R.block<3, 3>(rindex2, rindex2) = (1. + (1. - trust) * high_suspect_number) * R.block<3, 3>(rindex2, rindex2); //v
      R(rindex3, rindex3) = (1. + (1. - trust) * high_suspect_number) * R(rindex3, rindex3);                         //z

      trusts(i) = trust;
      //处理后的
      ps.segment(i1, 3) = -p_f;                                  //足端位置在世界坐标系描述
      vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f); //足端速度在世界坐标系描述
      pzs(i) = (1.0f - trust) * (p0(2) + p_f(2));
    }

    Vector28 y;

    y << ps, vs, pzs;

    //卡尔曼滤波
    xhat = A * xhat + B * acc; //状态预测方程

    MatrixX Pm = A * P * A.transpose() + Q; //不确定性预测方程

    //卡尔曼增益准备
    Vector28 yModel = C * xhat; //预测的观测值

    Vector28 ey = y - yModel; //误差 卡尔曼增益计算准备

    MatrixX S = C * Pm * C.transpose() + R; //卡尔曼增益计算准备

    // TODO compute LU only once
    Vector28 S_ey = S.lu().solve(ey); //求逆??

    xhat += Pm * C.transpose() * S_ey; //??

    MatrixX S_C = S.lu().solve(C); // ??  Eigen::Matrix<fpt_t, 28, 18>

    P = (Matrix18::Identity() - Pm * C.transpose() * S_C) * Pm; //最佳估计不确定性协方差??

    P = (P + P.transpose()) / 2.; //??

    if (P.block<2, 2>(0, 0).determinant() > 0.000001)
    { //??
      P.block<2, 16>(0, 2).setZero();
      P.block<16, 2>(2, 0).setZero();
      P.block<2, 2>(0, 0) /= 10.;
    }
    //输出状态量
    ToEigenTp(ret.pos) = xhat.block<3, 1>(0, 0);
    ToEigenTp(ret.vel) = xhat.block<3, 1>(3, 0);
    ToEigenTp(ret.vel_robot) = rot_mat * ToConstEigenTp(ret.vel);

    return true;
  }
}
