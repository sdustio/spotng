#pragma once
#include <memory>

#include "spotng/interface.h"

namespace spotng::fsm {
class LegCtrl {
 public:
  using Ptr = std::unique_ptr<LegCtrl>;
  using SharedPtr = std::shared_ptr<LegCtrl>;
  using ConstSharedPtr = std::shared_ptr<LegCtrl const>;

  explicit LegCtrl(interface::Leg::SharedPtr const &itf);
  interface::LegCmds cmds;
  bool RunOnce();
  void ZeroCmds();

 private:
  interface::Leg::SharedPtr const itf_;
};

}  // namespace spotng::fsm
