#pragma once

#include <memory>

#include "sdquadx/consts.h"
#include "sdquadx/types.h"

namespace sdquadx {

namespace options {

struct SDQUADX_EXPORT Model {
  Model();

  fpt_t body_length;
  fpt_t body_width;
  fpt_t body_height;

  fpt_t mass_body;
  fpt_t mass_abad;
  fpt_t mass_hip;
  fpt_t mass_knee;
  fpt_t mass_total;

  fpt_t link_length_abad;
  fpt_t link_length_hip;
  fpt_t link_length_knee;

  fpt_t basic_locomotion_height;
  fpt_t fast_locomotion_height;
  fpt_t foot_offsetx;
  fpt_t foot_offsety;

  SdVector3f location_abad_fl;
  SdVector3f location_hip_fl;
  SdVector3f location_knee_fl;

  SdVector3f com_body;
  SdVector3f com_abad_fl;
  SdVector3f com_hip_fl;
  SdVector3f com_knee_fl;

  SdMatrix3f inertia_body;
  SdMatrix3f inertia_abad;
  SdMatrix3f inertia_hip;
  SdMatrix3f inertia_knee;
  SdMatrix3f inertia_total;

  fpt_t max_com_height;
  fpt_t max_body_roll;
  fpt_t max_body_pitch;
  fpt_t max_body_yaw;
};

struct SDQUADX_EXPORT Ctrl {
  Ctrl();

  int mpc_iters;
  std::array<fpt_t, 13> mpc_weights;

  fpt_t footskd_bonus_swing;
  fpt_t footskd_vkp;

  SdVector3f kp_bodypos;
  SdVector3f kd_bodypos;

  SdVector3f kp_bodyori;
  SdVector3f kd_bodyori;

  SdVector3f kp_foot;
  SdVector3f kd_foot;

  SdVector3f kp_joint;
  SdVector3f kd_joint;

  SdVector3f kp_jpos;
  SdVector3f kd_jpos;

  std::array<SdVector3f, consts::model::kNumLeg> jpos_init;
  std::array<SdVector3f, consts::model::kNumLeg> jpos_fold;
  std::array<SdVector3f, consts::model::kNumLeg> jpos_stand;
  std::array<SdVector3f, consts::model::kNumLeg> jpos_rolling;

  fpt_t max_trot_lvel_x;
  fpt_t min_trot_lvel_x;
  fpt_t max_trot_lvel_y;
  fpt_t max_trot_avel_z;
};

struct SDQUADX_EXPORT Estimate {
  Estimate();

  fpt_t process_noise_pimu;
  fpt_t process_noise_vimu;
  fpt_t process_noise_pfoot;
  fpt_t sensor_noise_pfoot;
  fpt_t sensor_noise_vfoot;
  fpt_t sensor_noise_zfoot;
};

}  // namespace options

struct SDQUADX_EXPORT Options {
  using Ptr = std::unique_ptr<Options>;
  using SharedPtr = std::shared_ptr<Options>;
  using ConstSharedPtr = std::shared_ptr<Options const>;

  Options();

  fpt_t ctrl_sec;
  // gravity scalar
  fpt_t gravity;

  fpt_t rfmu;
  fpt_t rfmax;

  // debug, info, warn, err, critical
  char const* log_level;
  // console, file
  char const* log_target;
  char const* log_filename;

  options::Model model;
  options::Ctrl ctrl;
  options::Estimate estimate;
};

}  // namespace sdquadx
