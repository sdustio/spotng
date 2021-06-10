#include "sd/estimators/contact.h"

namespace sd::est
{

  bool Contact::Run(StateEst &ret, const Vector4d &contact_phase)
  {
    ret.contact_estimate = contact_phase;
    return true;
  }
}
