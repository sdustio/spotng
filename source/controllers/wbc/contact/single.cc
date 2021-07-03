#include "controllers/wbc/contact/single.h"

namespace sd::ctrl::wbc
{

  ContactSingle::ContactSingle(
      const dynamics::FBModelPtr &model, size_t contact_pt) : Contact(3), robot_sys_(model), _contact_pt(contact_pt)
  {
  }

  bool ContactSingle::_UpdateJc() { return true; }
  bool ContactSingle::_UpdateJcDotQdot() { return true; }
  bool ContactSingle::_UpdateUf() { return true; }
  bool ContactSingle::_UpdateInequalityVector() { return true; }

}
