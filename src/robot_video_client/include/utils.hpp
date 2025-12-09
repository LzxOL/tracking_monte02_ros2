/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-15 10:40:04
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-22 17:36:37
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#pragma once

#include <semaphore.h>
#include <stdint.h>
#include <sys/time.h>

#include <atomic>
#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

class TimePerf {
 public:
  TimePerf(const std::string& name = "Timer")
      : m_name(name), m_start(std::chrono::high_resolution_clock::now()) {}

  ~TimePerf() {
    auto end = std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - m_start)
            .count();
    std::cout << m_name << " elapsed time: " << duration << " ms" << std::endl;
  }

 private:
  std::string m_name;
  std::chrono::time_point<std::chrono::high_resolution_clock> m_start;
};

void saveYUV420M(const std::string& filename, uint8_t* yuv, int32_t width,
                 int32_t height);

void convertRGB2YUV420M(const uint8_t* rgb, uint8_t* yuv, int32_t width,
                        int32_t height);

void displayRGBFrame(uint8_t* data, int32_t width, int32_t height);

void displayPNGFrame(uint8_t* data, int32_t width, int32_t height);

void displayDepthImage(uint16_t* depthData, int width, int height);

uint64_t getStartupTimeMs();

uint64_t getCurrentTimeMs();

uint64_t getCurrentTimeUs();

int64_t getCurrentTimestampMs();
