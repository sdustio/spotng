#include "estimate/orientation.h"
#include "dynamics/rotation.h"

namespace sdrobot::estimate
{
  bool Orientation::Update(sensor::ImuData const &imu)
  {
    imu_ = imu;
    return true;
  }

  bool Orientation::RunOnce(State &ret)
  {
    //复制四元数值
    ret.ori = imu_.quat;

    Eigen::Map<Vector4> ori(ret.ori.data());
    Eigen::Map<Vector4> ori_ini_inv(ori_ini_inv_.data());

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

    Eigen::Map<Vector3> ros_rpy(ret.pos_rpy.data());
    dynamics::QuatToRPY(ros_rpy, ori); //转欧拉角

    Eigen::Map<dynamics::RotMat> rot_body(ret.rot_body.data());
    dynamics::QuatToRotMat(rot_body, ori); //转旋转矩阵

    ret.vel_rpy_body = imu_.gyro; //得机体坐标角速度

    Eigen::Map<Vector3> vel_rpy_body(ret.vel_rpy_body.data());
    Eigen::Map<Vector3> vel_rpy_world(ret.vel_rpy_world.data());
    vel_rpy_world = rot_body.transpose() * vel_rpy_body; //得世界坐标下角速度

    ret.acc_body = imu_.acc; //得机体坐标加速度

    Eigen::Map<Vector3> acc_body(ret.acc_body.data());
    Eigen::Map<Vector3> acc_world(ret.acc_world.data());
    acc_world = rot_body.transpose() * acc_body; //得世界坐标加速度
    return true;
  }
}
