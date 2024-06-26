#pragma once

#include <memory>

#include "spotng/options.h"

namespace spotng::drive {

enum class Mode : uint8_t { Auto, Manual };

enum class State : uint8_t { Init, RecoveryStand, Locomotion, BalanceStand };

enum class Gait : uint8_t { Trot, SlowTrot, FlyingTrot, Walk, Bound };

struct SPOTNG_EXPORT Twist {
  fpt_t lvel_x = 0.;
  fpt_t lvel_y = 0.;
  fpt_t lvel_z = 0.;  // [reserved, unused]
  fpt_t avel_x = 0.;  // [reserved, unused] roll vel
  fpt_t avel_y = 0.;  // [reserved, unused] pitch vel
  fpt_t avel_z = 0.;  // yaw vel
};

struct SPOTNG_EXPORT Pose {
  fpt_t roll = 0.;    // roll
  fpt_t pitch = 0.;   // pitch
  fpt_t yaw = 0.;     // yaw
  fpt_t height = 0.;  // height
};

class SPOTNG_EXPORT DriveCtrl {
 public:
  using Ptr = std::unique_ptr<DriveCtrl>;
  using SharedPtr = std::shared_ptr<DriveCtrl>;
  using ConstSharedPtr = std::shared_ptr<DriveCtrl const>;

  virtual ~DriveCtrl() = default;

  virtual bool UpdateMode(Mode const &mode) = 0;
  virtual bool UpdateTwist(Twist const &twist) = 0;
  virtual bool UpdatePose(Pose const &varpose) = 0;
  virtual bool UpdateState(State const &state) = 0;
  virtual bool UpdateGait(Gait const &gait) = 0;
  virtual bool UpdateStepHeight(fpt_t const height) = 0;

  virtual bool CmdtoDesData() = 0;

  virtual fpt_t GetStepHeight() const = 0;
  virtual State GetState() const = 0;
  virtual Gait GetGait() const = 0;

  virtual SdVector3f const &GetPosDes() const = 0;
  virtual SdVector3f const &GetRpyDes() const = 0;
  virtual SdVector3f const &GetLvelDes() const = 0;
  virtual SdVector3f const &GetAvelDes() const = 0;
};

}  // namespace spotng::drive
