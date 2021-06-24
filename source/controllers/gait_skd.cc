#include "sd/robot/runner.h"
#include "sd/robot/model.h"
#include "sd/controllers/gait_skd.h"

namespace sd::ctrl
{
  /**
 * Reset gait data to zeros
 */
  void GaitData::Zero()
  {
    // Stop any gait transitions
    next_gait = current_gait;

    // General Gait descriptors
    period_time_nominal = 0.0;     // overall period time to scale
    initial_phase = 0.0;           // initial phase to offset
    switching_phase_nominal = 0.0; // nominal phase to switch contacts

    // Enable flag for each foot
    gait_enabled = Eigen::Vector4i::Zero(); // enable gait controlled legs

    // Time based descriptors
    period_time = Vector4d::Zero();           // overall gait period time
    time_stance = Vector4d::Zero();           // total stance time
    time_swing = Vector4d::Zero();            // total swing time
    time_stance_remaining = Vector4d::Zero(); // stance time remaining
    time_swing_remaining = Vector4d::Zero();  // swing time remaining

    // Phase based descriptors
    switching_phase = Vector4d::Zero(); // phase to switch to swing
    phase_variable = Vector4d::Zero();  // overall gait phase for each foot
    phase_offset = Vector4d::Zero();    // nominal gait phase offsets
    phase_scale = Vector4d::Zero();     // phase scale relative to variable
    phase_stance = Vector4d::Zero();    // stance subphase
    phase_swing = Vector4d::Zero();     // swing subphase

    // Scheduled contact states
    contact_state_scheduled = Eigen::Vector4i::Zero(); // contact state of the foot
    contact_state_prev = Eigen::Vector4i::Zero();      // previous contact state of the foot
    touchdown_scheduled = Eigen::Vector4i::Zero();     // scheduled touchdown flag
    liftoff_scheduled = Eigen::Vector4i::Zero();       // scheduled liftoff flag
  }

  /*========================= Gait Scheduler ============================*/

  /**
 * Constructor to automatically setup a basic gait
 */
  GaitSkd::GaitSkd() : dt_(robot::ctrlparams::kCtrlsec),
                       create_gait_methods_{
                           &GaitSkd::CreateGaitStand,
                           &GaitSkd::CreateGaitTrot}
  {
    Initialize();
  }

  bool GaitSkd::SetNextGait(const GaitType gait)
  {
    gait_data_.next_gait = gait;
    return true;
  }

  const GaitData &GaitSkd::GetGaitData() const
  {
    return gait_data_;
  }

  /**
 * Initialize the gait data
 */
  void GaitSkd::Initialize()
  {
    // Start the gait in a trot since we use this the most 开始步态在小跑，因为我们使用这个最多
    gait_data_.current_gait = GaitType::STAND;

    // Create the gait from the nominal initial 创建步态从标称初始值
    CreateGait();
    period_time_natural = gait_data_.period_time_nominal;
    switching_phase_natural = gait_data_.switching_phase_nominal;
  }

  /**
 * Executes the Gait Schedule step to calculate values for the defining
 * gait parameters.
 */
  void GaitSkd::Step()
  {

    // Modify the gait with settings 修改步态设置（切换步态）
    ModifyGait();

    //非站立
    if (gait_data_.current_gait != GaitType::STAND)
    {
      // Track the reference phase variable 跟踪参考相位变量
      gait_data_.initial_phase = fmod((gait_data_.initial_phase + (dt_ / gait_data_.period_time_nominal)), 1);
    }

    // Iterate over the feet 遍历四足
    for (int foot = 0; foot < robot::ModelAttrs::num_leg; foot++)
    {
      // Set the previous contact state for the next timestep 设置前一个接触状态
      gait_data_.contact_state_prev(foot) = gait_data_.contact_state_scheduled(foot);

      if (gait_data_.gait_enabled(foot) == 1)
      {
        // Monotonic time based phase incrementation 单调时基相位增量
        if (gait_data_.current_gait == GaitType::STAND)
        {
          // Don't increment the phase when in stand mode 在站立模式下，不要增加相位
          dphase_ = 0.0;
        }
        else
        {
          dphase_ = gait_data_.phase_scale(foot) * (dt_ / gait_data_.period_time_nominal); //每次循环相位增量
        }

        // Find each foot's current phase 找到脚的当前相位 通过增加相位增量 并取余数来实现0~1
        gait_data_.phase_variable(foot) =
            fmod((gait_data_.phase_variable(foot) + dphase_), 1);

        // Check the current contact state 检测当前接触状态
        if (gait_data_.phase_variable(foot) <= gait_data_.switching_phase(foot)) //当前为接触状态
        {
          // Foot is scheduled to be in contact  足部预定进行接触
          gait_data_.contact_state_scheduled(foot) = 1;

          // Stance subphase calculation  在支撑相中的相位
          gait_data_.phase_stance(foot) =
              gait_data_.phase_variable(foot) / gait_data_.switching_phase(foot);

          // Swing phase has not started since foot is in stance  摇摆阶段还没有开始，因为脚在支撑
          gait_data_.phase_swing(foot) = 0.0;

          // Calculate the remaining time in stance 计算剩余支撑时间
          gait_data_.time_stance_remaining(foot) =
              gait_data_.period_time(foot) *
              (gait_data_.switching_phase(foot) - gait_data_.phase_variable(foot));

          // Foot is in stance, no swing time remaining 剩余摆动时间 当前腿在支撑 没有剩余摆动时间
          gait_data_.time_swing_remaining(foot) = 0.0;

          // First contact signifies scheduled touchdown  第一次接触表示预定着陆
          if (gait_data_.contact_state_prev(foot) == 0) //之前为摆动
          {
            // Set the touchdown flag to 1 设置着陆标志为1
            gait_data_.touchdown_scheduled(foot) = 1;
          }
          else
          {
            // Set the touchdown flag to 0 设置着陆标志为0
            gait_data_.touchdown_scheduled(foot) = 0;
          }
        }
        else
        {
          // Foot is not scheduled to be in contact 足端没有预定进行接触
          gait_data_.contact_state_scheduled(foot) = 0;

          // Stance phase has completed since foot is in swing  支撑相结束 在支撑相中的相位
          gait_data_.phase_stance(foot) = 1.0;

          // Swing subphase calculation  Swing相中的相位计算
          gait_data_.phase_swing(foot) =
              (gait_data_.phase_variable(foot) - gait_data_.switching_phase(foot)) /
              (1.0 - gait_data_.switching_phase(foot));

          // Foot is in swing, no stance time remaining 摆动没有支撑时间剩余
          gait_data_.time_stance_remaining(foot) = 0.0;

          // Calculate the remaining time in swing 计算剩余摆动时间
          gait_data_.time_swing_remaining(foot) =
              gait_data_.period_time(foot) * (1 - gait_data_.phase_variable(foot));

          // First contact signifies scheduled touchdown  第一次接触表示预定着陆
          if (gait_data_.contact_state_prev(foot) == 1) //之前为接触
          {
            // Set the liftoff flag to 1 设置离地标志为1
            gait_data_.liftoff_scheduled(foot) = 1;
          }
          else
          {
            // Set the liftoff flag to 0 设置离地标志为1
            gait_data_.liftoff_scheduled(foot) = 0;
          }
        }
      }
      else
      {
        // Leg is not enabled 腿未启用
        gait_data_.phase_variable(foot) = 0.0;

        // Foot is not scheduled to be in contact 脚没有接触计划
        gait_data_.contact_state_scheduled(foot) = 0;
      }
    }
  }

  /**
 *步态切换
 */
  void GaitSkd::ModifyGait()
  {
    // Use NaturalGaitModification from FSM_State 使用fsm来的设定
    if (gait_data_.current_gait != gait_data_.next_gait)
    {
      CreateGait();
      period_time_natural = gait_data_.period_time_nominal;
      switching_phase_natural = gait_data_.switching_phase_nominal;
    }
    else
    {
      gait_data_.period_time_nominal = period_time_natural;
      gait_data_.switching_phase_nominal = switching_phase_natural;
      CalcAuxiliaryGaitData();
    }
  }

  /**
 * Creates the gait structure from the important defining parameters of each
 * gait 为每种创建步态通过加载的参数
 *
 * To create a standard gait you should only need to define the following:
 * 为零创建标准步态，你只需要定义如下参数
 *
 *   gait_data_.period_time_nominal
 *   gait_data_.switchingPhaseNominal
 *   gait_data_.phase_offset
 *
 * The rest can be set to:
 * 其余的可以设置为：
 *
 *   gait_data_.gait_enabled << 1, 1, 1, 1;
 *   gait_data_.initial_phase = 0.0;
 *   gait_data_.phase_scale << 1.0, 1.0, 1.0, 1.0;
 *
 * These add flexibility to be used for very irregular gaits and transitions.
 * 这些增加了灵活性，以用于非常不规则的步态和过渡
 */
  void GaitSkd::CreateGait()
  {

    (this->*create_gait_methods_[size_t(gait_data_.next_gait)])();

    // Gait has switched
    gait_data_.current_gait = gait_data_.next_gait;

    // Calculate the auxilliary gait information
    CalcAuxiliaryGaitData();
  }

  void GaitSkd::CreateGaitStand()
  {
    gait_data_.gait_name = "STAND";
    gait_data_.gait_enabled << 1, 1, 1, 1;
    gait_data_.period_time_nominal = 10.0;
    gait_data_.initial_phase = 0.0;
    gait_data_.switching_phase_nominal = 1.0;
    gait_data_.phase_offset << 0.5, 0.5, 0.5, 0.5;
    gait_data_.phase_scale << 1.0, 1.0, 1.0, 1.0;
  }

  void GaitSkd::CreateGaitTrot()
  {
    gait_data_.gait_name = "TROT";
    gait_data_.gait_enabled << 1, 1, 1, 1;
    gait_data_.period_time_nominal = 0.5;
    gait_data_.initial_phase = 0.0;
    gait_data_.switching_phase_nominal = 0.5;
    gait_data_.phase_offset << 0.0, 0.5, 0.5, 0.0;
    gait_data_.phase_scale << 1.0, 1.0, 1.0, 1.0;
  }

  void GaitSkd::CalcAuxiliaryGaitData()
  {

    // Set the gait parameters for each foot
    for (int foot = 0; foot < robot::ModelAttrs::num_leg; foot++)
    {
      if (gait_data_.gait_enabled(foot) == 1)
      {
        // The scaled period time for each foot 每只脚的缩放周期时间
        gait_data_.period_time(foot) =
            gait_data_.period_time_nominal / gait_data_.phase_scale(foot);

        // Phase at which to switch the foot from stance to swing //将脚从站姿转换成摇摆姿势的阶段
        gait_data_.switching_phase(foot) = gait_data_.switching_phase_nominal;

        // Initialize the phase variables according to offset 根据偏移量初始化相位变量
        gait_data_.phase_variable(foot) =
            gait_data_.initial_phase + gait_data_.phase_offset(foot);

        // Find the total stance time over the gait cycle 找出整个步态周期的总站立时间
        gait_data_.time_stance(foot) =
            gait_data_.period_time(foot) * gait_data_.switching_phase(foot);

        // Find the total swing time over the gait cycle 找出整个步态周期的摆动时间
        gait_data_.time_swing(foot) =
            gait_data_.period_time(foot) * (1.0 - gait_data_.switching_phase(foot));
      }
      else
      {
        // The scaled period time for each foot
        gait_data_.period_time(foot) = 0.0;

        // Phase at which to switch the foot from stance to swing
        gait_data_.switching_phase(foot) = 0.0;

        // Initialize the phase variables according to offset
        gait_data_.phase_variable(foot) = 0.0;

        // Foot is never in stance
        gait_data_.time_stance(foot) = 0.0;

        // Foot is always in "swing"
        gait_data_.time_swing(foot) = 1.0 / gait_data_.period_time(foot);
      }
    }
  }

}
