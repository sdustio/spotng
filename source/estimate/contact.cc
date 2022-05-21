#include "estimate/contact.h"

namespace forax::estimate {
bool Contact::UpdateContact(SdVector4f const &contact) {
  contact_ = contact;
  return true;
}

bool Contact::RunOnce(State &ret) {
  ret.contact = contact_;
  return true;
}
}  // namespace forax::estimate
