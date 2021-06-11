#include "sd/estimators/pos_vel.h"
#include "sd/robot/runner.h"

namespace sd::est
{

  bool PosVel::Setup()
  {
    double dt = robot::ctrlparams::kCtrlsec;
    _xhat.setZero(); //状态估计值
    _ps.setZero();
    _vs.setZero();
    //状态转移矩阵，计算K+1时刻状态值X[k+1] 自己写出来算下就知道
    _A.setZero();
    _A.block<3, 3>(0, 0) = Matrix3d::Identity();
    _A.block<3, 3>(0, 3) = dt * Matrix3d::Identity();
    _A.block<3, 3>(3, 3) = Matrix3d::Identity();
    _A.block<12, 12>(6, 6) = Matrix12d::Identity();
    //输入矩阵
    _B.setZero();
    _B.block<3, 3>(3, 0) = dt * Matrix3d::Identity();
    //观测矩阵
    //观测量[p1 p2 p3 p4 v1 v2 v3 v4 z1 z2 z3 z4]（v z都在世界坐标下p在机身坐标系） p v 是向量 z是标量 pib=pb-piw vi=vb
    MatrixXd C1(3, 6);
    C1 << Matrix3d::Identity(), Matrix3d::Zero();
    MatrixXd C2(3, 6);
    C2 << Matrix3d::Zero(), Matrix3d::Identity();
    _C.setZero();
    _C.block<3, 6>(0, 0) = C1;
    _C.block<3, 6>(3, 0) = C1;
    _C.block<3, 6>(6, 0) = C1;
    _C.block<3, 6>(9, 0) = C1;
    _C.block<12, 12>(0, 6) = -1. * Matrix12d::Identity();
    _C.block<3, 6>(12, 0) = C2;
    _C.block<3, 6>(15, 0) = C2;
    _C.block<3, 6>(18, 0) = C2;
    _C.block<3, 6>(21, 0) = C2;
    _C(27, 17) = 1.;
    _C(26, 14) = 1.;
    _C(25, 11) = 1.;
    _C(24, 8) = 1.;

    //初始不确定性
    _P.setIdentity();
    _P = 100. * _P;
    //初始状态估计噪声
    _Q0.setIdentity();
    _Q0.block<3, 3>(0, 0) = (dt / 20.) * Matrix3d::Identity();
    _Q0.block<3, 3>(3, 3) =
        (dt * 9.8 / 20.) * Matrix3d::Identity();

    _Q0.block<12, 12>(6, 6) = dt * Matrix12d::Identity();

    _R0.setIdentity();

    return true;
  }

  bool PosVel::Run(StateEst &ret, const robot::leg::Datas &datas, const robot::QuadrupedPtr &quad)
  {

    double process_noise_pimu = robot::ctrlparams::kIMUProcessNoisePosition;
    double process_noise_vimu = robot::ctrlparams::kIMUProcessNoiseVelocity;
    double process_noise_pfoot = robot::ctrlparams::kFootProcessNoisePosition;
    double sensor_noise_pimu_rel_foot = robot::ctrlparams::kFootSensorNoisePosition;
    double sensor_noise_vimu_rel_foot = robot::ctrlparams::kFootSensorNoiseVelocity;
    double sensor_noise_zfoot = robot::ctrlparams::kFootHeightSensorNoise;
    //状态估计噪声
    Matrix18d Q = Matrix18d::Identity();
    Q.block<3, 3>(0, 0) = _Q0.block<3, 3>(0, 0) * process_noise_pimu;
    Q.block<3, 3>(3, 3) = _Q0.block<3, 3>(3, 3) * process_noise_vimu;
    Q.block<12, 12>(6, 6) = _Q0.block<12, 12>(6, 6) * process_noise_pfoot;
    //观测噪声矩阵
    Matrix28d R = Matrix28d::Identity();
    R.block<12, 12>(0, 0) = _R0.block<12, 12>(0, 0) * sensor_noise_pimu_rel_foot;
    R.block<12, 12>(12, 12) =
        _R0.block<12, 12>(12, 12) * sensor_noise_vimu_rel_foot;
    R.block<4, 4>(24, 24) = _R0.block<4, 4>(24, 24) * sensor_noise_zfoot;

    int qindex = 0;
    int rindex1 = 0;
    int rindex2 = 0;
    int rindex3 = 0;
    //重力向量
    Vector3d g(0, 0, -9.81);
    Matrix3d Rbod = ret.rot_body.transpose(); //机身到世界的变换矩阵
    // in old code, Rbod * se_acc + g
    //输入量a 世界下
    Vector3d a = ret.a_world + g;

    // std::cout << "A WORLD\n" << a << "\n";
    Vector4d pzs = Vector4d::Zero();
    Vector4d trusts = Vector4d::Zero();
    Vector3d p0, v0;
    //初始位置 速度
    p0 << _xhat[0], _xhat[1], _xhat[2];
    v0 << _xhat[3], _xhat[4], _xhat[5];

    //构成状态变量等
    for (int i = 0; i < robot::ModelAttrs::num_leg; i++)
    {
      int i1 = 3 * i;
      Vector3d ph = quad->GetHipLocation(i); // hip positions relative to CoM 相对于CoM的髋位置
      // hw_i->leg_controller->leg_datas[i].p;
      Vector3d p_rel = ph + datas[i].p; //足端位置在机身坐标系
      // hw_i->leg_controller->leg_datas[i].v;
      Vector3d dp_rel = datas[i].v; //足端速度在机身坐标系
      Vector3d p_f = Rbod * p_rel;                                        //足端位置在世界坐标系描述 即方向 大小 没有位置

      //足端速度在世界坐标系描述 机身转动导致足端速度+足端本身速度
      Vector3d dp_f =
          Rbod * (ret.omega_body.cross(p_rel) + dp_rel);

      //更新四条腿用索引
      qindex = 6 + i1;
      rindex1 = i1;      //p
      rindex2 = 12 + i1; //V
      rindex3 = 24 + i;  //Z

      double trust = 1.;
      double phase = fmin(ret.contact_estimate(i), 1.); //获得接触状态估计
                                                                                  //获取接触状态 在整个支撑过程百分比(从0到1)  完成后为0
                                                                                  //脚i的测量协方差在摆动过程中被提高到一个很高的值，因此在这个融合过程中，摆动腿的测量值被有效地忽略了
      //double trust_window = double(0.25);
      double trust_window = 0.2;

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
      //double high_suspect_number(1000);
      double high_suspect_number = 100.;

      // printf("Trust %d: %.3f\n", i, trust);
      //摆动腿和支撑腿刚触地，即将离地时状态，测量噪声协方差增大
      Q.block<3, 3>(qindex, qindex) =
          (1. + (1. - trust) * high_suspect_number) * Q.block<3, 3>(qindex, qindex); //p

      R.block<3, 3>(rindex1, rindex1) = 1 * R.block<3, 3>(rindex1, rindex1);                                             //p
      R.block<3, 3>(rindex2, rindex2) = (1. + (1. - trust) * high_suspect_number) * R.block<3, 3>(rindex2, rindex2); //v
      R(rindex3, rindex3) = (1. + (1. - trust) * high_suspect_number) * R(rindex3, rindex3);                         //z

      trusts(i) = trust;
      //处理后的
      _ps.segment(i1, 3) = -p_f;                                  //足端位置在世界坐标系描述
      _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f); //足端速度在世界坐标系描述
      pzs(i) = (1.0f - trust) * (p0(2) + p_f(2));
    }

    Vector28d y;

    y << _ps, _vs, pzs;

    //卡尔曼滤波
    _xhat = _A * _xhat + _B * a; //状态预测方程

    Matrix18d At = _A.transpose();

    Matrix18d Pm = _A * _P * At + Q; //不确定性预测方程

    //卡尔曼增益准备
    Eigen::Matrix<double, 18, 28> Ct = _C.transpose();

    Vector28d yModel = _C * _xhat; //预测的观测值

    Vector28d ey = y - yModel; //误差 卡尔曼增益计算准备

    Matrix28d S = _C * Pm * Ct + R; //卡尔曼增益计算准备

    // todo compute LU only once
    Vector28d S_ey = S.lu().solve(ey); //求逆？？

    _xhat += Pm * Ct * S_ey; //？？

    Eigen::Matrix<double, 28, 18> S_C = S.lu().solve(_C); //？?

    _P = (Matrix18d::Identity() - Pm * Ct * S_C) * Pm; //最佳估计不确定性协方差??

    Matrix18d Pt = _P.transpose(); //??
    _P = (_P + Pt) / 2.;                        //??

    if (_P.block<2, 2>(0, 0).determinant() > 0.000001)
    { //??
      _P.block<2, 16>(0, 2).setZero();
      _P.block<16, 2>(2, 0).setZero();
      _P.block<2, 2>(0, 0) /= 10.;
    }
    //输出状态量
    ret.position = _xhat.block<3, 1>(0, 0);
    ret.v_world = _xhat.block<3, 1>(3, 0);
    ret.v_body = ret.rot_body * ret.v_world;

    return true;
  }
}
