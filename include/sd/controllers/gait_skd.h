#pragma once

#include <memory>

#include "sd/types.h"

namespace sd::ctrl
{
  /**
 * Enumerated gait types. Preplanned gaits are defined.
 * 枚举类型
 */
  enum class GaitType : uint8_t
  {
    STAND,
    STAND_CYCLE,
    STATIC_WALK,
    AMBLE,
    TROT_WALK,
    TROT,
    TROT_RUN,
    PACE,
    BOUND,
    ROTARY_GALLOP,
    TRAVERSE_GALLOP,
    PRONK,
    THREE_FOOT,
    CUSTOM,
    TRANSITION_TO_STAND
  };

  /**
 * Timing data for a gait 步态时间信息
 */
  struct GaitData
  {
    GaitData() { Zero(); }

    // Zero out all of the data
    void Zero();

    // The current GaitType 当前GaitType
    GaitType current_gait;

    // Next GaitType to transition into 下一步GaitType过渡到
    GaitType next_gait;

    // Gait name string 步态的名字字符串
    std::string gait_name;

    // Gait descriptors 步态描述符
    double period_time_nominal;     // overall period time to scale 总周期时间以规模
    double initial_phase;           // initial phase to offset 初始偏移相位
    double switching_phase_nominal; // nominal phase to switch contacts 标称相位切换点

    // Enable flag for each foot 启用每只脚的标记
    Eigen::Vector4i gait_enabled; // enable gait controlled legs 启动步态控制腿

    // Time based descriptors 基于时间的描述符
    Vector4d period_time;           // overall foot scaled gait period time 整个步态时间
    Vector4d time_stance;           // total stance time 总站立时间
    Vector4d time_swing;            // total swing time 总摆动时间
    Vector4d time_stance_remaining; // stance time remaining 剩余站立时间
    Vector4d time_swing_remaining;  // swing time remaining 剩余摆动时间

    // Phase based descriptors 阶段基于描述符
    Vector4d switching_phase; // phase to switch to swing 相切换到swing
    Vector4d phase_variable;  // overall gait phase for each foot 每只脚的整体步态阶段
    Vector4d phase_offset;    // nominal gait phase offsets 步态相位偏移
    Vector4d phase_scale;     // phase scale relative to variable 相对于变量的相位刻度
    Vector4d phase_stance;    // stance subphase 支撑相中相位
    Vector4d phase_swing;     // swing subphase 摆动相中相位

    // Scheduled contact states 预定的接触状态
    Eigen::Vector4i contact_state_scheduled; // contact state of the foot 脚的接触状态
    Eigen::Vector4i contact_state_prev;      // previous contact state of the foot 脚的先前接触状态
    Eigen::Vector4i touchdown_scheduled;     // scheduled touchdown event flag 预定的触地事件标志
    Eigen::Vector4i liftoff_scheduled;       // scheduled lift event flag 预定离地事件标值
  };

  /**
 * Utility to process GaitData and schedule foot steps and swings.
 * 实用程序，以处理步态数据和计划的步伐和摆动。
 */
  class GaitSkd
  {
  public:
    // Constructors for the GaitSkd
    GaitSkd(); //构造函数

    // Iteration step for scheduler logic 逻辑循环
    void Step();

    bool SetNextGait(const GaitType gait);

    const GaitData& GetGaitData() const;

  private:
    // Initialize the Gait Scheduler//初始化
    void Initialize();

    // Creates a new gait from predefined library
    //修改步态
    void ModifyGait();

    //创建步态
    void CreateGait();

    //计算辅助步态数据
    void CalcAuxiliaryGaitData();

    // Struct containing all of the gait relevant data 储存所有步态相关数据
    GaitData gait_data_;

    // Control loop timestep change
    double dt_;

    // Phase change at each step
    double dphase_;

    // Natural gait modifiers  自然步态修饰符
    //周期时间
    double period_time_natural = 0.5;
    //摆动向接触切换点
    double switching_phase_natural = 0.5;
    //摆动时间
    double swing_time_natural = 0.25;
  };

  using GaitSkdPtr = std::unique_ptr<GaitSkd>;
  using GaitSkdSharedPtr = std::shared_ptr<GaitSkd>;
}
