#pragma once

#include <memory>

#include "sd/estimators/state_est.h"

namespace sd::est
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
    bool Run(StateEst &ret, const Vector4d &contact_phase);
  };

  using ContactPtr = std::unique_ptr<Contact>;
  using ContactSharedPtr = std::shared_ptr<Contact>;

}
