#pragma once

#include <memory>

#include "sd/robot/model.h"
#include "sd/robot/interface.h"

namespace sd::ctrl
{

  class LegCtrl
  {
  public:
    void SetLegEnabled(bool enabled) { enabled_ = enabled; }

    const robot::leg::Datas& GetDatas() const { return datas_; }

    robot::leg::Cmds& GetCmdsForUpdate() { return cmds_; }

    /*!
    * Update the "leg data" from a SPIne board message
    * 从spine卡 更新腿部信息
    */
    void UpdateData(const robot::SPIData &data);

    /*!
    * Update the "leg command" for the SPIne board message
    * 向控制器发送控制命令
    */
    void UpdateSPICmd(robot::SPICmd &cmd);

    /*!
    * Zero all leg commands.  This should be run *before* any control code, so if
    * the control code is confused and doesn't change the leg command, the legs
    * won't remember the last command.
    * 腿部控制命令清零，应运行在任何控制代码之前，否则控制代码混乱，控制命令不会改变，腿部不会记忆上次命令
    */
    void ZeroCmd();

    /*!
    * Compute the position of the foot and its Jacobian.  This is done in the local
    * leg coordinate system. If J/p are NULL, the calculation will be skipped.
    */
    void ComputeLegJacobianAndPosition(int leg);

  private:
    robot::leg::Cmds cmds_;
    robot::leg::Datas datas_;

    bool enabled_ = false;
  };

  using LegCtrlPtr = std::unique_ptr<LegCtrl>;
  using LegCtrlSharedPtr = std::shared_ptr<LegCtrl>;
}
