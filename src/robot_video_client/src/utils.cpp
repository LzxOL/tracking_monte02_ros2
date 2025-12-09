/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-21 01:26:33
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-22 17:36:17
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */

#include "utils.hpp"

#include <opencv2/opencv.hpp>

void saveYUV420M(const std::string& filename, uint8_t* yuv, int32_t width,
                 int32_t height) {
  std::ofstream file(filename, std::ios::binary);
  if (file.is_open()) {
    file.write(reinterpret_cast<char*>(yuv), width * height * 3 / 2);
    file.close();
    std::cout << "YUV420M 数据已保存至：" << filename << std::endl;
  } else {
    std::cerr << "无法打开文件：" << filename << std::endl;
  }
}

void convertRGB2YUV420M(const uint8_t* rgb, uint8_t* yuv, int32_t width,
                        int32_t height) {
  // TimePerf t("RGB8_to_YUV420M");

  int32_t frameSize = width * height;
  int32_t uvIndex = frameSize;                   // U 分量的起始位置
  int32_t vIndex = frameSize + (frameSize / 4);  // V 分量的起始位置

  for (int32_t j = 0; j < height; j++) {
    for (int32_t i = 0; i < width; i++) {
      int32_t rgbIndex = (j * width + i) * 3;
      uint8_t R = rgb[rgbIndex];
      uint8_t G = rgb[rgbIndex + 1];
      uint8_t B = rgb[rgbIndex + 2];

      // 计算 Y 分量
      yuv[j * width + i] =
          static_cast<uint8_t>(0.299 * R + 0.587 * G + 0.114 * B);

      // 只对偶数行和偶数列采样 U 和 V
      if (j % 2 == 0 && i % 2 == 0) {
        int32_t avgIndex = (j / 2) * (width / 2) + (i / 2);
        uint8_t Y = yuv[j * width + i];

        // 计算 U 分量
        yuv[uvIndex + avgIndex] = static_cast<uint8_t>((B - Y) * 0.565 + 128);

        // 计算 V 分量
        yuv[vIndex + avgIndex] = static_cast<uint8_t>((R - Y) * 0.713 + 128);
      }
    }
  }
}

void displayRGBFrame(uint8_t* data, int32_t width, int32_t height) {
  // 创建一个OpenCV的Mat对象
  cv::Mat rgbImage(height, width, CV_8UC3, data);

  // 转换为BGR格式（OpenCV默认的显示格式）
  cv::Mat bgrImage;
  cv::cvtColor(rgbImage, bgrImage, cv::COLOR_RGB2BGR);

  // 显示图像
  cv::imshow("Decoded Video Frame", bgrImage);
  cv::waitKey(1);  // 等待1毫秒
}

void displayPNGFrame(uint8_t* data, int32_t width, int32_t height) {
  // Convert the raw PNG data to a cv::Mat (image)
  cv::Mat img =
      cv::imdecode(cv::Mat(1, width * height, CV_8U, data), cv::IMREAD_COLOR);

  if (img.empty()) {
    std::cerr << "Error: Unable to decode the PNG image." << std::endl;
    return;
  }

  // Show the image in a window
  cv::imshow("PNG Frame", img);

  // Wait for a key press indefinitely
  cv::waitKey(1);  // 0 means it will wait indefinitely until a key is pressed
}

void displayDepthImage(uint16_t* depthData, int width, int height) {
  // 将 depthData 转换为 cv::Mat 类型，使用 16 位无符号整数格式
  cv::Mat depthMat(height, width, CV_16UC1, depthData);

  // 归一化深度图像，将深度值映射到 0-255 范围
  cv::Mat depthDisplay;
  double minVal, maxVal;
  cv::minMaxLoc(depthMat, &minVal, &maxVal);  // 获取最小值和最大值
  depthMat.convertTo(depthDisplay, CV_8UC1, 255.0 / (maxVal - minVal),
                     -minVal);  // 归一化到 0-255

  // 使用伪彩色显示深度图
  cv::Mat colorDepth;
  cv::applyColorMap(depthDisplay, colorDepth,
                    cv::COLORMAP_JET);  // 使用 JET 色彩映射

  // 显示图像
  cv::imshow("Depth Image", colorDepth);
  cv::waitKey(1);
}

uint64_t getStartupTimeMs() {
  struct timespec ts;
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
    perror("clock_gettime");
    return 0;
  }
  return (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

uint64_t getCurrentTimeMs() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return (uint64_t)t.tv_sec * 1000 + t.tv_usec / 1000;
}

uint64_t getCurrentTimeUs() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return (uint64_t)t.tv_sec * 1000 * 1000 + t.tv_usec;
}

int64_t getCurrentTimestampMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}