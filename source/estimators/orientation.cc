#include "sd/estimators/orientation.h"
#include "sd/dynamics/rotation.h"


namespace sd::estimators
{
  bool Orientation::Run(StateEst& ret, const robot::IMUData& imu)
  {
	//复制四元数值
  ret.orientation = imu.quat; // 和 microstrainImu 顺序相同

//初值
  if(_b_first_visit){
    Vector3d rpy_ini = dynamics::QuatToRPY(ret.orientation); //四元数转欧拉角
    rpy_ini[0] = 0;//roll  pitch 设零
    rpy_ini[1] = 0;
    _ori_ini_inv = dynamics::RPYToQuat(-rpy_ini); //初始四元数的逆
    _b_first_visit = false;
  }

  ret.orientation = dynamics::QuatProduct(_ori_ini_inv, ret.orientation); //两四元数相乘

  ret.rpy = dynamics::QuatToRPY(ret.orientation);//转欧拉角


  ret.rot_body = dynamics::QuatToRotMat(ret.orientation);//转旋转矩阵

  ret.omega_body = imu.gyro.template cast<double>();//得机体下角速度

  ret.omega_world = ret.rot_body.transpose() * ret.omega_body;//得世界下角速度

  ret.a_body = imu.acc.template cast<double>();//得加速度
  ret.a_world = ret.rot_body.transpose() * ret.a_body;

  return true;
  }
}