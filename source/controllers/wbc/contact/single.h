#pragma once

#include "sd/controllers/wbc.h"

namespace sd::ctrl::wbc
{
  class ContactSingle : public Contact
  {
  public:
    ContactSingle(const dynamics::FBModelPtr& model, size_t contact_pt);

  private:
    bool _UpdateJc() override;
    bool _UpdateJcDotQdot() override;
    bool _UpdateUf() override;
    bool _UpdateInequalityVector() override;

    const dynamics::FBModelPtr robot_sys_;
    size_t _contact_pt;
  };

}
