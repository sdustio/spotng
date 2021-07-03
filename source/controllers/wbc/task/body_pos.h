#pragma once

#include "sd/controllers/wbc.h"

namespace sd::ctrl::wbc
{
  class TaskBodyPos : public Task
  {
  public:
    TaskBodyPos(const dynamics::FBModelPtr& model);
  private:
    /* data */
  };

}
