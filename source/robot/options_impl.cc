#include "spotng/options.h"

namespace spotng {

namespace options {

Model::Model()
    : body_length(0.58),
      body_width(0.132),
      body_height(0.099),

      mass_body(25.),
      mass_abad(2.),
      mass_hip(1.),
      mass_knee(1.),
      mass_total(41.),

      link_length_abad(0.093),
      link_length_hip(0.284),
      link_length_knee(0.284),

      basic_locomotion_height(0.4),
      fast_locomotion_height(0.34),
      foot_offsetx(-0.03),
      foot_offsety(-0.02),

      location_abad_fl({0.29, 0.066, 0.}),
      location_hip_fl({0.05, 0.093, 0.}),
      location_knee_fl({0., 0., -0.284}),

      com_body({0., 0., 0.}),
      com_abad_fl({0.05, 0.021, 0.}),
      com_hip_fl({0., 0., -0.142}),
      com_knee_fl({0., 0., -0.142}),

      inertia_body({0.0567188, 0., 0., 0., 0.721252, 0., 0., 0., 0.737133}),
      inertia_abad({0.002426, 0., 0., 0., 0.0025, 0., 0., 0., 0.002426}),
      inertia_hip({0.00679633, 0., 0., 0., 0.00682342, 0., 0., 0., 0.000177083}),
      inertia_knee({0.00679633, 0., 0., 0., 0.00682342, 0., 0., 0., 0.000177083}),
      inertia_total({0.07487, 0., 0., 0., 2.1566, 0., 0., 0., 2.1775}),

      max_com_height(0.55),
      max_body_roll(0.523),
      max_body_pitch(0.785),
      max_body_yaw(0.523) {}

Ctrl::Ctrl()
    : mpc_iters(15),  // 30ms, 30/(ctrl_sec * 1000)
      mpc_weights({1.25, 1.25, 10., 2., 2., 50., 0., 0., 0.3, 1.5, 1.5, 0.2, 0.}),

      footskd_bonus_swing(0.),
      footskd_vkp(0.1),

      kp_bodypos({100., 100., 100.}),
      kd_bodypos({10., 10., 20.}),

      kp_bodyori({100., 100., 100.}),
      kd_bodyori({10., 10., 10.}),

      kp_foot({500., 500., 500.}),
      kd_foot({60., 60., 60.}),

      kp_joint({3., 3., 3.}),
      kd_joint({1., 0.2, 0.2}),

      kp_jpos({80., 80., 80.}),
      kd_jpos({1., 1., 1.}),

      jpos_init({-0., 1.4, -2.7, 0., 1.4, -2.7, -0., 1.4, -2.7, 0., 1.4, -2.7}),

      jpos_fold({-0., 1.4, -2.7, 0., 1.4, -2.7, -0., 1.4, -2.7, 0., 1.4, -2.7}),
      jpos_stand({-0., 0.8, -1.6, 0., 0.8, -1.6, -0., 0.9, -1.5, 0., 0.9, -1.5}),
      jpos_rolling({1.5, 1.6, -2.77, 1.3, 3.1, -2.77, 1.5, 1.6, -2.77, 1.3, 3.1, -2.77}),

      max_trot_lvel_x(1.8),
      min_trot_lvel_x(-0.8),
      max_trot_lvel_y(0.4),
      max_trot_avel_z(0.8) {}

Estimate::Estimate()
    : process_noise_pimu(0.02),
      process_noise_vimu(0.02),
      process_noise_pfoot(0.002),
      sensor_noise_pfoot(0.001),
      sensor_noise_vfoot(0.1),
      sensor_noise_zfoot(0.001) {}

}  // namespace options

Options::Options()
    : ctrl_sec(1.0 / (0.5 * 1'000)),  // 0.5kH

      gravity(9.81),
      rfmu(0.4),
      rfmax(1500),

      log_level(logging::Level::Warn),
      log_target(logging::Target::Console),
      log_filename("log/spotng.log"),
      log_max_file_size(1073741824),  // 1GiB
      log_max_files(3) {}

}  // namespace spotng
