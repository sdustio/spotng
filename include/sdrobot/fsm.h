#pragma once

#include <memory>

#include "sdrobot/drive.h"
#include "sdrobot/estimate.h"
#include "sdrobot/leg.h"
#include "sdrobot/model.h"


namespace sdrobot::fsm
{
  /**
    * Enumerate all of the operating modes
    */
  enum class SDROBOT_EXPORT OperatingMode : uint8_t
  {
    Normal,
    Transitioning,
    EStop,
  };

  using State = drive::State;

  struct SDROBOT_EXPORT TransitionData
  {
    // Flag to mark when transition is done
    bool done = false;
  };

  class SDROBOT_EXPORT StateCtrl
  {
  public:
    using Ptr = std::unique_ptr<StateCtrl>;
    using SharedPtr = std::shared_ptr<StateCtrl>;

    virtual ~StateCtrl() = default;

    // Behavior to be carried out when entering a state
    virtual void OnEnter() = 0;

    // Behavior to be carried out when exiting a state
    virtual void OnExit() = 0;

    // Run the normal behavior for the state
    virtual bool Run() = 0;

    // Manages state specific transitions
    virtual State CheckTransition() = 0;

    // Runs the transition behaviors and returns true when done transitioning
    virtual TransitionData Transition(const State next) = 0;

    // Return State Enum
    virtual State GetState() const = 0;

    // Pre controls safety checks
    virtual bool NeedCheckSafeOrientation() const { return false; }

    // Post control safety checks
    virtual bool NeedCheckPDesFoot() const { return false; }
    virtual bool NeedCheckForceFeedForward() const { return false; }
  };

  class SDROBOT_EXPORT FiniteStateMachine
  {
  public:
    /**
     * Called each control loop iteration. Decides if the robot is safe to
     * run controls and checks the current state for any transitions. Runs
     * the regular state behavior if all is normal.
     */
    virtual bool RunOnce() = 0;
    virtual bool UpdateLegCtrl(leg::LegCtrl::SharedPtr const &legctrl) = 0;
    virtual bool UpdateQuadruped(model::Quadruped::SharedPtr const &quad) = 0;
    virtual bool UpdateDriveCtrl(drive::DriveCtrl::SharedPtr const &drivectrl) = 0;
    virtual bool UpdateEstimateCtrl(estimate::EstimateCtrl::SharedPtr const &estctrl) = 0;
  };
}
