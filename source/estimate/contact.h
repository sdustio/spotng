#pragma once

#include "forax/estimate.h"

namespace forax::estimate {
class Contact : public Estimator {
 public:
  bool UpdateContact(SdVector4f const &contact);
  bool RunOnce(State &ret) override;

 private:
  SdVector4f contact_ = {0.5, 0.5, 0.5, 0.5};
};

}  // namespace forax::estimate
