/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-28 13:51:18
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-21 10:29:11
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#pragma once

#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

using StreamDataCallback = std::function<void(
    uint8_t *data, uint32_t len, uint32_t width, uint32_t height,
    uint64_t timestamp, uint32_t type, uint32_t id)>;

using StreamEventCallback =
    std::function<void(int32_t event_id, void *event_data, uint32_t id)>;