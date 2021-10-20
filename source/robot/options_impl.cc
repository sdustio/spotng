#include "sdquadx/options.h"

namespace sdquadx {

namespace options {

Model::Model()
    : body_length(0.9),
      body_width(0.25),
      body_height(0.1),

      mass_body(28.),
      mass_rotor(0.055),
      mass_abad(0.5),
      mass_hip(0.55),
      mass_knee(0.05),

      gear_ratio_abad(7.5),
      gear_ratio_hip(7.5),
      gear_ratio_knee(9.33),

      link_length_abad(0.1125),
      link_length_hip(0.35),
      link_length_knee(0.3),
      link_yoffset_knee(0.),

      location_abad_fl({0.32, 0.0625, 0.}),
      location_abad_rotor_fl({0.25, 0.0625, 0.}),
      location_hip_fl({0., 0.1125, 0.}),
      location_hip_rotor_fl({0., 0.0375, 0.}),
      location_knee_fl({0., 0., -0.3}),
      location_knee_rotor_fl({0., 0., 0.}),

      com_body({0., 0., 0.}),
      com_rotor({0., 0., 0.}),
      com_abad_l({0., 0., 0.}),
      com_hip_l({0., -0.0175, -0.02}),
      com_knee_l({0., 0., -0.061}),

      inertia_body({0.169167, 0, 0, 0, 1.91333, 0, 0, 0, 2.03583}),
      inertia_rotor_z({3.3e-5, 0, 0, 0, 3.3e-5, 0, 0, 0, 6.3e-5}),
      inertia_abad({8.46875e-4, 0., 0., 0., 1.225e-3, 0., 0., 0., 8.46875e-4}),
      inertia_hip({8.46875e-4, 0., 0., 0., 1.225e-3, 0., 0., 0., 8.46875e-4}),
      inertia_knee({3.76667e-3, 0., 0., 0., 3.85417e-4, 0., 0., 0., 1.20833e-5}) {}

Ctrl::Ctrl()
    : kp_body({100., 100., 100.}),
      kd_body({10., 10., 20.}),

      kp_foot({500., 500., 500.}),
      kd_foot({60., 60., 60.}),

      kp_ori({100., 100., 100.}),
      kd_ori({10., 10., 10.}),
      kp_joint({3., 3., 3.}),
      kd_joint({1., 0.2, 0.2}),

      kp_jpos({60., 60., 60.}),
      kd_jpos({1., 1., 1.}),

      jpos_init({-0., -1.40335, 2.97414, 0., -1.40335, 2.97414, -0., -1.40335, 2.97414, 0., -1.40335, 2.97414}),

      jpos_fold({-0., -1.4, 2.7, 0.0, -1.4, 2.7, -0.0, -1.4, 2.7, 0.0, -1.4, 2.7}),
      jpos_stand({-0., -0.8, 1.6, 0., -0.8, 1.6, -0., -0.8, 1.6, 0., -0.8, 1.6}),
      jpos_rolling({1.5, -1.6, 2.77, 1.3, -3.1, 2.77, 1.5, -1.6, 2.77, 1.3, -3.1, 2.77}) {}

}  // namespace options

Options::Options()
    : drive_mode(options::DriveMode::kAuto),

      ctrl_sec(1.0 / (0.5 * 1'000)),      // 0.5kH
      act_itf_sec(1.0 / (0.04 * 1'000)),  // 0.04kH

      jpos_init_sec(3.),
      gravity(9.81),

      log_level("warn"),
      log_target("console"),
      log_filename("log/sdquadx.log") {}

}  // namespace sdquadx
