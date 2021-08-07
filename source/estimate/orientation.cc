#include "estimate/orientation.h"
#include "dynamics/rotation.h"

namespace sdrobot::estimate
{
  bool Orientation::UpdateImu(sensor::ImuData const &imu)
  {
    imu_ = imu;
    return true;
  }

  bool Orientation::RunOnce(State &ret)
  {
    //复制四元数值
    ret.ori = imu_.quat;

    auto ori = ToEigenTp(ret.ori);
    auto ori_ini_inv = ToEigenTp(ori_ini_inv_);

    if (b_first_visit_)
    {
      Vector3 rpy_ini;
      dynamics::QuatToRPY(rpy_ini, ori); //四元数转欧拉角
      rpy_ini[0] = 0;                    //roll  pitch 设零
      rpy_ini[1] = 0;
      dynamics::RPYToQuat(ori_ini_inv, -rpy_ini); //初始四元数的逆
      b_first_visit_ = false;
    }

    dynamics::QuatProduct(ori, ori_ini_inv, ori); //两四元数相乘

    dynamics::QuatToRPY(ToEigenTp(ret.pos_rpy), ori); //转欧拉角

    dynamics::QuatToRotMat(ToEigenTp(ret.rot_body), ori); //转旋转矩阵
    ret.vel_rpy_body = imu_.gyro; //得机体坐标角速度

    ToEigenTp(ret.vel_rpy_world) = ToConstEigenTp(ret.rot_body).transpose() * ToConstEigenTp(ret.vel_rpy_body); //得世界坐标下角速度

    ret.acc_body = imu_.acc; //得机体坐标加速度

    ToEigenTp(ret.acc_world) = ToConstEigenTp(ret.rot_body).transpose() * ToConstEigenTp(ret.acc_body); //得世界坐标加速度
    return true;
  }
}
