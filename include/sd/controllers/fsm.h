#pragma once

#include <memory>
#include <array>

#include "sd/types.h"
#include "sd/robot/interface.h"

namespace sd::ctrl
{
  namespace fsm
  {

    /**
    * Enumerate all of the operating modes
    */
    enum class OperatingMode : size_t
    {
      Normal,
      Transitioning,
      EStop,
    };

    enum class State : uint8_t
    {
      Off,
      Ready,
      StandUp,
      Locomotion,
      RecoveryStand,
      BalanceStand,
      Count_
    };

    struct TransitionData
    {
      // Flag to mark when transition is done
      bool done = false;
    };

    class StateCtrl
    {
    public:
      // Behavior to be carried out when entering a state
      virtual void OnEnter() = 0;

      // Behavior to be carried out when exiting a state
      virtual void OnExit() = 0;

      // Run the normal behavior for the state
      virtual bool Run() = 0;

      // Manages state specific transitions
      virtual State CheckTransition() = 0;

      // Runs the transition behaviors and returns true when done transitioning
      virtual TransitionData Transition() = 0;

      // Return State Enum
      virtual State GetState() const = 0;

      // Pre controls safety checks
      virtual bool NeedCheckSafeOrientation() const = 0;

      // Post control safety checks
      virtual bool NeedCheckPDesFoot() const = 0;
      virtual bool NeedCheckForceFeedForward() const = 0;
      virtual bool NeedCheckLegSingularity() const = 0;
    };

    using StateCtrlPtr = std::shared_ptr<StateCtrl>;

    class SafetyChecker
    {
    public:
      bool PreCheck(const StateCtrlPtr &ctrl, robot::Mode mode);
      bool PostCheck(const StateCtrlPtr &ctrl, robot::Mode mode);

    private:
      // Pre checks to make sure controls are safe to run
      bool CheckSafeOrientation(); // robot's orientation is safe to control

      // Post checks to make sure controls can be sent to robot
      bool CheckPDesFoot();         // desired foot position is not too far
      bool CheckForceFeedForward(); // desired feedforward forces are not too large
    };

  } // namespace fsm

  class FSM
  {
  public:
    FSM();

    /**
    * Initialize the Control FSM with the default settings. SHould be set to
    * Passive state and Normal operation mode.
    */
    bool Init();

    /**
     * Called each control loop iteration. Decides if the robot is safe to
     * run controls and checks the current state for any transitions. Runs
     * the regular state behavior if all is normal.
     */
    bool Run(robot::Mode mode);

  private:
    fsm::StateCtrlPtr GetStateCtrl(fsm::State state);
    std::array<fsm::StateCtrlPtr, size_t(fsm::State::Count_)> state_ctrls_;
    fsm::StateCtrlPtr current_state_ctrl_;
    fsm::StateCtrlPtr next_state_ctrl_;

    fsm::State next_state_;
    fsm::OperatingMode opmode_;
    robot::Mode ctrl_mode_;

    fsm::SafetyChecker safety_checker_;
    fsm::TransitionData transition_data_;
  };

} // namespace sd::ctrl
