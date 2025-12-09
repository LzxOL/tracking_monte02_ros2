#ifndef _TIMER_HPP
#define _TIMER_HPP

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <future>
#include <iostream>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

class CTask {
 public:
  CTask(int32_t iId, std::function<void()> funCb, uint32_t u32Interval,
        int32_t iLoopCnt, std::chrono::steady_clock::time_point expireTime)
      : m_iId(iId),
        m_funCb(funCb),
        m_u32Interval(u32Interval),
        m_iLoopCnt(iLoopCnt),
        m_expireTime(expireTime) {}

  int32_t m_iId = -1;
  std::function<void()> m_funCb = nullptr;
  uint32_t m_u32Interval = 0;
  int32_t m_iLoopCnt = 0;
  std::chrono::steady_clock::time_point m_expireTime;

  // 自定义比较函数，按照到期时间从小到大排序
  struct Compare {
    bool operator()(const CTask& lhs, const CTask& rhs) const {
      return lhs.m_expireTime < rhs.m_expireTime;
    }
  };
};

class CTimer {
 public:
  /**
   * @brief 定时器构造函数.
   * @param bAsync 执行模式.
   *        false: 当有多个任务到期时依次执行任务，任务很轻量时推荐此模式.
   *        true:
   * 当有多个任务到期时异步并行执行任务，任务需要执行IO等耗时操作时推荐此模式.
   * @return 无.
   */

  explicit CTimer(bool bAsync = false)
      : m_bAsync(bAsync),
        m_bRunning(true),
        m_thrWorker(&CTimer::doWork, this){

        };

  virtual ~CTimer() { stop(); }

  /**
   * @brief 添加一个定时任务.
   * @param funCb 超时回调函数.
   * @param u32Interval 定时间隔，单位ms.
   * @param iLoopCnt 循环执行次数
   *                  <0: 执行次数不限.
   *                  >0: 执行的次数.
   * @return 成功返回任务ID,失败返回-1.
   */
  int32_t add_task(const std::function<void()>& funCb, uint32_t u32Interval,
                   int32_t iLoopCnt) {
    if (!funCb || iLoopCnt == 0) {
      std::cout << "Invalid parameter!" << std::endl;
      return -1;
    }

    auto id = ++m_iCount;
    auto expireTime = std::chrono::steady_clock::now() +
                      std::chrono::milliseconds(u32Interval);

    std::unique_lock<std::mutex> lock(m_mtxTask);
    m_tasks.emplace(id, funCb, u32Interval, iLoopCnt, expireTime);
    m_cv.notify_one();

    return id;
  }

  /**
   * @brief 删除一个定时任务.
   * @param iId 任务ID.
   * @return 无.
   */
  void remove_task(int32_t iId) {
    std::unique_lock<std::mutex> lock(m_mtxTask);

    auto it =
        std::find_if(m_tasks.begin(), m_tasks.end(),
                     [&](const CTask& task) { return task.m_iId == iId; });

    if (it != m_tasks.end()) {
      m_tasks.erase(it);
    }
  }

 protected:
  // 同步执行
  void syncExecutor(const std::vector<CTask>& vecTasks) {
    for (auto& task : vecTasks) {
      task.m_funCb();
    }
  }

  // 异步执行
  void asyncExecutor(const std::vector<CTask>& vecTasks) {
    std::vector<std::future<void>> futures;

    for (auto& task : vecTasks) {
      futures.push_back(std::async(std::launch::async, task.m_funCb));
    }

    for (auto& future : futures) {
      future.wait();
    }
  }

  // 执行任务
  void executeTasks(const std::vector<CTask>& vecTasks) {
    if (m_bAsync) {
      asyncExecutor(vecTasks);
    } else {
      syncExecutor(vecTasks);
    }
  }

  void doWork() {
    while (m_bRunning) {
      std::unique_lock<std::mutex> lock(m_mtxTask);

      if (m_tasks.empty()) {
        m_cv.wait(lock, [this] { return !m_tasks.empty() || !m_bRunning; });
      }

      if (!m_bRunning) {
        break;
      }

      // 找出待执行的任务
      std::vector<CTask> vecDueTasks;
      auto now = std::chrono::steady_clock::now();
      for (auto it = m_tasks.begin(); it != m_tasks.end();) {
        if (it->m_expireTime <= now) {
          vecDueTasks.push_back(*it);
          it = m_tasks.erase(it);
        } else {
          break;
        }
      }

      // 执行任务
      if (!vecDueTasks.empty()) {
        executeTasks(vecDueTasks);
      }

      // 重新装载任务
      for (auto& t : vecDueTasks) {
        if (t.m_iLoopCnt != 0) {
          if (t.m_iLoopCnt > 0) {
            t.m_iLoopCnt--;
          }

          t.m_expireTime = std::chrono::steady_clock::now() +
                           std::chrono::milliseconds(t.m_u32Interval);
          m_tasks.emplace(t);
        }
      }

      if (!m_tasks.empty()) {
        m_cv.wait_until(lock, m_tasks.begin()->m_expireTime,
                        [this] { return !m_bRunning; });
      }
    }
  }

  void stop() {
    if (m_bRunning) {
      m_bRunning = false;
      m_cv.notify_one();
      if (m_thrWorker.joinable()) {
        m_thrWorker.join();
      }
    }
  }

 protected:
  std::atomic_bool m_bAsync{false};
  std::atomic_int32_t m_iCount{0};
  std::mutex m_mtxTask;
  std::condition_variable m_cv;
  std::multiset<CTask, CTask::Compare> m_tasks;
  std::atomic_bool m_bRunning{false};
  std::thread m_thrWorker;
};

#endif