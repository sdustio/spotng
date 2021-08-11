#include "estimate/contact.h"

namespace sdrobot::estimate
{
  bool Contact::UpdateContact(SdVector4f const &contact)
  {
    contact_ = contact;
    return true;
  }

  bool Contact::RunOnce(State &ret)
  {
    ret.contact = contact_;
    return true;
  }
}