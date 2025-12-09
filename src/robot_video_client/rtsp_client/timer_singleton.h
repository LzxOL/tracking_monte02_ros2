/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-28 23:02:16
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-21 10:28:45
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#ifndef _TIMER_SINGLETON_H
#define _TIMER_SINGLETON_H

#include "timer.hpp"

class CTimerSingleton : public CTimer {
 public:
  static CTimerSingleton* getInstance() {
    static CTimerSingleton objTimer;
    return &objTimer;
  }
};

#endif