#include "estimate/orientation.h"

#include "dynamics/rotation.h"
#include "math/utils.h"

namespace sdengine::estimate {
Orientation::Orientation(interface::Imu::ConstSharedPtr const &itf) : itf_(itf) {}

bool Orientation::RunOnce(State &ret) {
  if (!InterfaceValid()) {
    ret.success = false;
    return false;
  }

  ret.ori = imu_.quat;
  auto ori = ToEigenTp(ret.ori);
  auto ori_ini_inv = ToEigenTp(ori_ini_inv_);

  if (b_first_visit_) {
    Vector3 rpy_ini;
    dynamics::QuatToRPY(rpy_ini, ori);  // 四元数转欧拉角
    rpy_ini[0] = 0;                     // roll  pitch 设零
    rpy_ini[1] = 0;
    dynamics::RPYToQuat(ori_ini_inv, -rpy_ini);  // 初始四元数的逆
    b_first_visit_ = false;
  }

  dynamics::QuatProduct(ori, ori_ini_inv, ori);  // 两四元数相乘

  dynamics::QuatToRPY(ToEigenTp(ret.rpy), ori);  // 转欧拉角

  dynamics::QuatToRotMat(ToEigenTp(ret.rot_mat), ori);  // 转旋转矩阵
  ret.avel_robot = imu_.gyro;                           // 得机体坐标角速度

  ToEigenTp(ret.avel) = ToConstEigenTp(ret.rot_mat) * ToConstEigenTp(ret.avel_robot);  // 得世界坐标下角速度

  ret.acc_robot = imu_.acc;  // 得机体坐标加速度

  ToEigenTp(ret.acc) = ToConstEigenTp(ret.rot_mat) * ToConstEigenTp(ret.acc_robot);  // 得世界坐标加速度

  ret.success = true;
  return true;
}

bool Orientation::InterfaceValid() {
  itf_->ReadTo(imu_);

  if (math::HasNaN(imu_.quat.cbegin(), imu_.quat.cend())) return false;
  if (math::HasNaN(imu_.gyro.cbegin(), imu_.gyro.cend())) return false;
  if (math::HasNaN(imu_.acc.cbegin(), imu_.acc.cend())) return false;
  return true;
}
}  // namespace sdengine::estimate
