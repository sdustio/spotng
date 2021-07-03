#pragma once

#include "sd/controllers/wbc.h"

namespace sd::ctrl::wbc
{
  class TaskBodyOri : public Task
  {
  public:
    TaskBodyOri(const dynamics::FBModelPtr& model);
  private:
    /* data */
  };

}
