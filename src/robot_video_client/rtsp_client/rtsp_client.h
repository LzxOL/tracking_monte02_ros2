/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-30 17:03:02
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-21 10:28:24
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#pragma once

#include "common_def.h"

class IRtspClientImpl;

class IRtspClient {
 public:
  IRtspClient(const std::string& url, StreamDataCallback callback,
              int32_t chn_id = -1);
  ~IRtspClient();

  bool start();
  void stop();

 private:
  IRtspClientImpl* m_impl_;
};