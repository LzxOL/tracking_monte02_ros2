/*
 * @Description: 
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-28 13:49:37
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-21 11:23:40
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#pragma once

#include "common_def.h"
#include "media_rtsp_client.h"
#include "timer_singleton.h"

class IRtspClientImpl {
public:
  IRtspClientImpl(const std::string& url, StreamDataCallback callback, int32_t chn_id);

  ~IRtspClientImpl();

  bool start();
  void stop();

private:
  bool openStream();
  void closeStream();
  void eventLoop();

private:
  int32_t m_chn_id_ = 0;
  std::string m_url_;
  StreamDataCallback m_callback_ = nullptr;

  bool m_running_ = false;
  std::shared_ptr<std::thread> m_thread_;

  TaskScheduler* m_scheduler_ = nullptr;
  UsageEnvironment* m_env_ = nullptr;
  EventLoopWatchVariable m_watch_var_  = 0;
  RTSPClient *m_rtsp_client_ = nullptr;
  std::atomic_bool m_b_stream_open_ = false;

  std::atomic<int64_t> m_last_frame_time_;

  int32_t m_timer_task_id_ = -1;
};