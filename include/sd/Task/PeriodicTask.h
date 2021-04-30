#pragma once

#include <string>
#include <thread>
#include <vector>
#include <chrono>

namespace sd::task
{

    class PeriodicTaskManager;

    /*!
 * A single periodic task which will call run() at the given frequency
 * 一个周期任务，它将以给定的频率调用run() run()被定义在继承的子类中
 */
    class PeriodicTask
    {
    public:
      PeriodicTask(PeriodicTaskManager *taskManager, std::chrono::nanoseconds period,
                   std::string name); //构造时会将构造的任务添加到任务管理器 taskManager
      void start();                   //开始运行的任务 开一个线程运行执行周期运行程序
      void stop();                    //停止运行的任务
      void printStatus();             //打印运行状态
      void clearMax();                //清除最大值 下面的几个max
      bool isSlow();                  //是否慢速运行
      //纯虚函数只有函回数的名字而不具备函数的功能，不能被调用。它只是通知编译系统:

      //  “在这里声明一个虚函数，留待派生类中定义”。在答派生类中对此函数提供定义后，它才能具备函数的功能，可被调用。
      virtual void init() = 0;
      virtual void run() = 0;
      virtual void cleanup() = 0;
      virtual ~PeriodicTask() { stop(); }

      /*!
   * Get the desired period for the task 获得任务所需的周期
   */
      std::chrono::nanoseconds getPeriod() { return _period; }

      /*!
   * Get how long the most recent run took 了解最近一次跑步花了多长时间
   */
      std::chrono::nanoseconds getRuntime() { return _lastRuntime; }

      /*!
   * Get the maximum time in between runs 获取运行之间的最大时间
   */
      std::chrono::nanoseconds getMaxPeriod() { return _maxPeriodTime; }

      /*!
   * Get the maximum time it took for a run 获得运行所需的最大时间
   */
      std::chrono::nanoseconds getMaxRuntime() { return _maxRuntime; }

    private:
      void loopFunction(); //运行代码的地方 以一个固定周期运行
      double nsToSecF(std::chrono::nanoseconds d);

      std::chrono::nanoseconds _period; //周期
      std::chrono::nanoseconds _lastRuntime = std::chrono::nanoseconds::zero();
      std::chrono::nanoseconds _lastPeriodTime = std::chrono::nanoseconds::zero();
      std::chrono::nanoseconds _maxPeriodTime = std::chrono::nanoseconds::zero();
      std::chrono::nanoseconds _maxRuntime = std::chrono::nanoseconds::zero();

      volatile bool _running = false;
      std::string _name;   //任务名
      std::thread _thread; //任务线程
    };

    /*!
 * A collection of periodic tasks which can be monitored together
 * 可以一起监视的定期任务的集合
 */
    class PeriodicTaskManager
    {
    public:
      PeriodicTaskManager() = default;
      ~PeriodicTaskManager();
      void addTask(PeriodicTask *task);
      void printStatus();
      void printStatusOfSlowTasks();
      void stopAll();

    private:
      std::vector<PeriodicTask *> _tasks;
    };

    /*!
 * A periodic task for calling a function
 * 用于调用函数的周期性任务
 */
    class PeriodicCallFunction : public PeriodicTask
    {
    public:
      PeriodicCallFunction(PeriodicTaskManager *taskManager, std::chrono::nanoseconds period,
                           std::string name, void (*function)())
          : PeriodicTask(taskManager, period, name), _function(function) {}
      void cleanup() {}
      void init() {}
      void run() { _function(); }

      ~PeriodicCallFunction() { stop(); }

    private:
      void (*_function)() = nullptr;
    };


 /*!
 * A periodic task for calling a member function
 * 用于调用成员函数的周期性任务
 */
    template <typename T>
    class PeriodicCallMemberFunction : public PeriodicTask
    {
    public:
      PeriodicCallMemberFunction(PeriodicTaskManager *taskManager, chrono::nanoseconds period,
                                 string name, void (T::*function)(), T *obj)
          : PeriodicTask(taskManager, period, name),
            _function(function),
            _obj(obj) {}

      void cleanup() {}
      void init() {}
      void run() { (_obj->*_function)(); }

    private:
      void (T::*_function)();
      T *_obj;
    };

} // namespace sd::task
