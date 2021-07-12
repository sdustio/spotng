#pragma once

#include <memory>

#include "sdrobot/estimators/state_est.h"

namespace sdrobot::est
{

  /*!
  * A "passthrough" contact estimator which returns the expected contact state
  */
  class Contact
  {
  public:
    /*!
    * Set the estimated contact by copying the exptected contact state into the
    * estimated contact state
    */
    bool Run(StateData &ret, const Vector4d &contact_phase);
  };

  using ContactPtr = std::shared_ptr<Contact>;

}
