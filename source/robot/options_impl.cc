#include "sdquadx/options.h"

namespace sdquadx {

Options::Options()
    : drive_mode(DriveMode::kAuto),

      ctrl_sec(1.0 / (0.5 * 1'000)),      // 0.5kH
      act_itf_sec(1.0 / (0.04 * 1'000)),  // 0.04kH

      jpos_init_sec(3.),
      gravity(9.81),

      log_level("warn"),
      log_target("console"),
      log_filename("log/sdquadx.log"),

      kp_joint({3, 3, 3}),
      kd_joint({1, 0.2, 0.2}),

      kp_body({100, 100, 100}),
      kd_body({10, 10, 20}),

      kp_foot({500, 500, 500}),
      kd_foot({60, 60, 60}),

      kp_ori({100, 100, 100}),
      kd_ori({10, 10, 10}),

      kp_st({80, 80, 80}),
      kd_st({1, 1, 1}),

      init_jpos({-0., -1.40335, 2.97414, 0., -1.40335, 2.97414, -0., -1.40335, 2.97414, 0., -1.40335, 2.97414}),

      fold_jpos({-0., -1.4, 2.7, 0.0, -1.4, 2.7, -0.0, -1.4, 2.7, 0.0, -1.4, 2.7}),
      stand_jpos({-0., -0.8, 1.6, 0., -0.8, 1.6, -0., -0.8, 1.6, 0., -0.8, 1.6}),
      rolling_jpos({1.5, -1.6, 2.77, 1.3, -3.1, 2.77, 1.5, -1.6, 2.77, 1.3, -3.1, 2.77}) {}
}  // namespace sdquadx
