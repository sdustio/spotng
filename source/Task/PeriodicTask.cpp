/*!
 *  * @file PeriodicTask.cpp
 *  * @brief Implementation of a periodic function running in a separate thread.
 *  * Periodic tasks have a task manager, which measure how long they take to run.
 *  * 在单独线程中运行的周期函数的实现。周期性任务有一个任务管理器，用于度量它们运行所需的时间。
 *  */

#include "sd/Task/PeriodicTask.h"

namespace sd::task
{
		using namespace std;

		/*!
			 *  * Construct a new task within a TaskManager
			 *  * 在TaskManager中构造一个新任务
			 *  * @param taskManager : Parent task manager
			 *  * @param period : how often to run 多久跑一次
			 *  * @param name : name of task 任务名称
			 *  */
		PeriodicTask::PeriodicTask(PeriodicTaskManager *taskManager, chrono::nanoseconds period,
								   string name)
			: _period(period), _name(name)
		{
			taskManager->addTask(this); //向任务管理器添加新任务
		}

		/*!
		 *  * Begin running task 开始运行的任务 开一个线程运行执行周期运行程序
		 *  */
		void PeriodicTask::start()
		{
			if (_running)
			{
				printf("[PeriodicTask] Tried to start %s but it was already running!\n",
					   _name.c_str());
				return;
			}
			init();
			_running = true;
			_thread = thread(&PeriodicTask::loopFunction, this); //开一个线程运行执行周期运行程序
		}

		/*!
		 *  * Stop running task 停止运行的任务
		 *  */
		void PeriodicTask::stop()
		{
			if (!_running)
			{
				printf("[PeriodicTask] Tried to stop %s but it wasn't running!\n",
					   _name.c_str());
				return;
			}
			_running = false;
			printf("[PeriodicTask] Waiting for %s to stop...\n", _name.c_str());
			_thread.join(); //调用该函数会阻塞当前线程。阻塞调用者(caller)所在的线程直至被join的thread对象标识的线程执行结束。
			printf("[PeriodicTask] Done!\n");
			cleanup();
		}

		/*!
		 *  * If max period is more than 30% over desired period, it is slow
		 *   如果最大周期超过期望周期的30%，则速度较慢
		 *  */
		bool PeriodicTask::isSlow()
		{
			return _maxPeriodTime > _period * 1.3 || _maxRuntime > _period;
		}

		/*!
		 *  * Reset max statistics 重置最大值
		 *  */
		void PeriodicTask::clearMax()
		{
			_maxPeriodTime = chrono::nanoseconds::zero();
			_maxRuntime = chrono::nanoseconds::zero();
		}

		/*!
		 *  * Print the status of this task in the table format
		 *   以表格格式打印此任务的状态
		 *  */
		void PeriodicTask::printStatus()
		{
			if (!_running)
				return;
			printf("|%-20s|%6.4f|%6.4f|%6.4f|%6.4f|%6.4f\n", _name.c_str(),
				   nsToSecF(_lastRuntime), nsToSecF(_maxRuntime),
				   nsToSecF(_period), nsToSecF(_lastPeriodTime), nsToSecF(_maxPeriodTime));
		}

		double PeriodicTask::nsToSecF(chrono::nanoseconds d){
			return d.count() / 1e9;
		}

		/*!
		 *  * Call the task in a timed loop.  Uses a timerfd
		 *  * 在定时循环中调用任务。使用一个timerfd
		 *  */
		void PeriodicTask::loopFunction()
		{
			printf("[PeriodicTask] Start %s (%6.4f s)\n", _name.c_str(), nsToSecF(_period));

			auto last_tp = chrono::steady_clock::now();

			while (_running)
			{
				auto now = chrono::steady_clock::now();
				_lastPeriodTime = now - last_tp;
				last_tp = now;
				run();
				now = chrono::steady_clock::now();
				_lastRuntime = now - last_tp;
				last_tp = now;
				_maxPeriodTime = max(_maxPeriodTime, _lastPeriodTime);
				_maxRuntime = max(_maxRuntime, _lastRuntime);
			}
			printf("[PeriodicTask] %s has stopped!\n", _name.c_str());
		}

		PeriodicTaskManager::~PeriodicTaskManager() {}

		/*!
		 *  * Add a new task to a task manager
		 *  * 向任务管理器添加新任务
		 *  */
		void PeriodicTaskManager::addTask(PeriodicTask *task)
		{
			_tasks.push_back(task);
		}

		/*!
		 *  * Print the status of all tasks and rest max statistics
		 *  打印所有任务的状态和重置 max统计数据
		 *  */
		void PeriodicTaskManager::printStatus()
		{
			printf("\n----------------------------TASKS----------------------------\n");
			printf("|%-20s|%-6s|%-6s|%-6s|%-6s|%-6s\n", "name", "rt", "rt-max", "T-des",
				   "T-act", "T-max");
			printf("-----------------------------------------------------------\n");
			for (auto &task : _tasks)
			{
				task->printStatus();
				task->clearMax();
			}
			printf("-------------------------------------------------------------\n\n");
		}

		/*!
		 *  * Print only the slow tasks
		 *  只打印缓慢的任务
		 *  */
		void PeriodicTaskManager::printStatusOfSlowTasks()
		{
			for (auto &task : _tasks)
			{
				if (task->isSlow())
				{
					task->printStatus();
					task->clearMax();
				}
			}
		}

		/*!
		 *  * Stop all tasks 停止所有的任务
		 *  */
		void PeriodicTaskManager::stopAll()
		{
			for (auto &task : _tasks)
			{
				task->stop();
			}
		}

} // namespace sd::task
