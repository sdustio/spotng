#pragma once

#include <memory>
#include <unordered_map>

#include "sdrobot/robot/interface.h"
#include "sdrobot/estimators/state_est.h"
#include "sdrobot/controllers/leg.h"
#include "sdrobot/controllers/state_cmd.h"

namespace sdrobot::ctrl
{
  namespace fsm
  {

    /**
    * Enumerate all of the operating modes
    */
    enum class OperatingMode : uint8_t
    {
      Normal,
      Transitioning,
      EStop,
    };

    using State = robot::Mode;

    struct TransitionData
    {
      // Flag to mark when transition is done
      bool done = false;
    };

    class StateCtrl
    {
    public:
      StateCtrl(const LegPtr &cleg, const robot::QuadrupedPtr &quad, const StateCmdPtr &cmd, const est::StateEstPtr &est);
      // Behavior to be carried out when entering a state
      virtual void OnEnter() = 0;

      // Behavior to be carried out when exiting a state
      virtual void OnExit() = 0;

      // Run the normal behavior for the state
      virtual bool Run() = 0;

      // Manages state specific transitions
      virtual State CheckTransition(const StateCmdPtr &cmd) = 0;

      // Runs the transition behaviors and returns true when done transitioning
      virtual TransitionData Transition(const State next) = 0;

      // Return State Enum
      virtual State GetState() const = 0;

      // Pre controls safety checks
      virtual bool NeedCheckSafeOrientation() const { return false; }

      // Post control safety checks
      virtual bool NeedCheckPDesFoot() const { return false; }
      virtual bool NeedCheckForceFeedForward() const { return false; }

    protected:
      LegPtr leg_ctrl_;
      robot::QuadrupedPtr quad_;
      const StateCmdPtr state_cmd_;
      const est::StateEstPtr state_est_;
    };

    using StateCtrlPtr = std::shared_ptr<StateCtrl>;

    class SafetyChecker
    {
    public:
      // Pre checks to make sure controls are safe to run
      bool CheckSafeOrientation(const est::StateData &est); // robot's orientation is safe to control

      // Post checks to make sure controls can be sent to robot
      bool CheckPDesFoot(LegPtr &cleg);         // desired foot position is not too far
      bool CheckForceFeedForward(LegPtr &cleg); // desired feedforward forces are not too large
    };

  } // namespace fsm

  class Fsm
  {
  public:
    Fsm(const LegPtr &cleg, const robot::QuadrupedPtr &quad, const StateCmdPtr &cmd, const est::StateEstPtr &est);

    /**
     * Called each control loop iteration. Decides if the robot is safe to
     * run controls and checks the current state for any transitions. Runs
     * the regular state behavior if all is normal.
     */
    bool Run();

  private:
    /**
    * Initialize the Control Fsm with the default settings. SHould be set to
    * Passive state and Normal operation mode.
    */
    bool Init();
    bool PreCheck();
    bool PostCheckAndLimit();

    LegPtr leg_ctrl_;
    robot::QuadrupedPtr quad_;
    const StateCmdPtr state_cmd_;
    const est::StateEstPtr state_est_;

    fsm::StateCtrlPtr GetStateCtrl(fsm::State state);
    std::unordered_map<fsm::State, fsm::StateCtrlPtr> state_ctrls_;
    fsm::StateCtrlPtr current_state_ctrl_;
    fsm::StateCtrlPtr next_state_ctrl_;

    fsm::State next_state_;
    fsm::OperatingMode opmode_;

    fsm::SafetyChecker safety_checker_;
    fsm::TransitionData transition_data_;
  };

  using FsmPtr = std::shared_ptr<Fsm>;

} // namespace sdrobot::ctrl