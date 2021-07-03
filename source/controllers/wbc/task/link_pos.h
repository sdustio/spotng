#pragma once

#include "sd/controllers/wbc.h"

namespace sd::ctrl::wbc
{
  class TaskLinkPos : public Task
  {
  public:
    TaskLinkPos(const dynamics::FBModelPtr& model);
  private:
    /* data */
  };

}
