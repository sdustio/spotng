#include "sdquadx/options.h"

namespace sdquadx {

namespace options {

Model::Model()
    : body_length(0.68),
      body_width(0.132),
      body_height(0.099),

      mass_body(25.),
      mass_abad(2.),
      mass_hip(1.),
      mass_knee(1.),

      link_length_abad(0.093),
      link_length_hip(0.284),
      link_length_knee(0.284),
      link_yoffset_knee(0.),

      location_abad_fl({0.39, 0.066, 0.}),
      location_hip_fl({0., 0.093, 0.}),
      location_knee_fl({0., 0., -0.284}),

      com_body({0., 0., 0.}),
      com_abad_fl({0., 0.021, 0.}),
      com_hip_fl({0., 0., -0.142}),
      com_knee_fl({0., 0., -0.142}),

      inertia_body({0.0567188, 0, 0, 0, 0.721252, 0, 0, 0, 0.737133}),
      inertia_abad({0.002426, 0., 0., 0., 0.0025, 0., 0., 0., 0.002426}),
      inertia_hip({0.00679633, 0., 0., 0., 0.00682342, 0., 0., 0., 0.000177083}),
      inertia_knee({0.00679633, 0., 0., 0., 0.00682342, 0., 0., 0., 0.000177083}) {}

Ctrl::Ctrl()
    : kp_body({100., 100., 100.}),
      kd_body({10., 10., 20.}),

      kp_foot({500., 500., 500.}),
      kd_foot({60., 60., 60.}),

      kp_ori({100., 100., 100.}),
      kd_ori({10., 10., 10.}),
      kp_joint({3., 3., 3.}),
      kd_joint({1., 0.2, 0.2}),

      kp_jpos({120., 120., 120.}),
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
