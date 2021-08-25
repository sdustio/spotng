#pragma once

#include "sdrobot/estimate.h"

namespace sdrobot::estimate {
class Contact : public Estimator {
 public:
  bool UpdateContact(SdVector4f const &contact);
  bool RunOnce(State &ret) override;

 private:
  SdVector4f contact_;
};

}  // namespace sdrobot::estimate
