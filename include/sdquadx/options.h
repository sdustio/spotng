#pragma once

#include <memory>
#include <string>

#include "sdquadx/consts.h"
#include "sdquadx/types.h"

namespace sdquadx {

enum class DriveMode : uint8_t {
  kAuto,
  kManual,
};

using JPosVectorf = std::array<SdVector3f, consts::model::kNumLeg>;

struct SDQUADX_EXPORT Options {
  using Ptr = std::unique_ptr<Options>;
  using SharedPtr = std::shared_ptr<Options>;
  using ConstSharedPtr = std::shared_ptr<Options const>;

  Options();

  DriveMode drive_mode;

  fpt_t ctrl_sec;
  fpt_t act_itf_sec;

  fpt_t jpos_init_sec;
  // gravity scalar
  fpt_t gravity;

  // debug, info, warn, err, critical
  std::string log_level;
  // console, file
  std::string log_target;
  std::string log_filename;

  SdVector3f kp_joint;
  SdVector3f kd_joint;

  SdVector3f kp_body;
  SdVector3f kd_body;

  SdVector3f kp_foot;
  SdVector3f kd_foot;

  SdVector3f kp_ori;
  SdVector3f kd_ori;

  SdVector3f kp_joint_flip;
  SdVector3f kd_joint_flip;

  JPosVectorf init_jpos;

  JPosVectorf fold_jpos;
  JPosVectorf stand_jpos;
  JPosVectorf rolling_jpos;
};

}  // namespace sdquadx
