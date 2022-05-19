#pragma once

#include "sdengine/estimate.h"

namespace sdengine::estimate {
class Contact : public Estimator {
 public:
  bool UpdateContact(SdVector4f const &contact);
  bool RunOnce(State &ret) override;

 private:
  SdVector4f contact_ = {0.5, 0.5, 0.5, 0.5};
};

}  // namespace sdengine::estimate
