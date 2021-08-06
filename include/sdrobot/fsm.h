#pragma once

#include "sdrobot/drive.h"

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
    using ConstSharedPtr = std::shared_ptr<StateCtrl const>;

    virtual ~StateCtrl() = default;

    // Behavior to be carried out when entering a state
    virtual bool OnEnter() = 0;

    // Behavior to be carried out when exiting a state
    virtual bool OnExit() = 0;

    // Run the normal behavior for the state
    virtual bool RunOnce() = 0;

    // Manages state specific transitions
    virtual State CheckTransition() = 0;

    // Runs the transition behaviors and returns true when done transitioning
    virtual TransitionData Transition(State const next) = 0;

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
    using Ptr = std::unique_ptr<FiniteStateMachine>;
    using SharedPtr = std::shared_ptr<FiniteStateMachine>;
    using ConstSharedPtr = std::shared_ptr<FiniteStateMachine const>;

    virtual ~FiniteStateMachine() = default;

    virtual StateCtrl::SharedPtr const &GetStateCtrl(State const state) = 0;

    /**
     * Called each control loop iteration. Decides if the robot is safe to
     * run controls and checks the current state for any transitions. Runs
     * the regular state behavior if all is normal.
     */
    virtual bool RunOnce() = 0;
  };
}
