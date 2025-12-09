/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-16 13:06:05
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-21 14:29:12
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

#include "depth_decoder.h"
#include "video_decoder.h"
#include "yhdds.h"
#include "yllog.h"


/*
struct VideoConfig_t {
  int32_t m_id;
  bool m_enable;
  std::string m_name;
  std::string m_url;
  std::string m_pub_topic_color_image_raw;
  std::string m_sub_topic_depth_image_compressed;
  std::string m_pub_topic_depth_image_raw;
};*/

// 新增流配置结构体
struct StreamConfig_t {
  std::string m_url;
  std::string m_pub_topic_color_image_raw;
};

struct VideoConfig_t {
  int32_t m_id;
  bool m_enable;
  std::string m_name;
  std::vector<StreamConfig_t> m_streams;  // 修改为流数组
  std::string m_sub_topic_depth_image_compressed;
  std::string m_pub_topic_depth_image_raw;
  
  // 保留原有字段以兼容旧配置
  std::string m_url;
  std::string m_pub_topic_color_image_raw;
};




class RobotVideoClient : public rclcpp::Node {
 public:
  RobotVideoClient();
  virtual ~RobotVideoClient();

  bool init();
  void deinit();

 private:
  bool loadVideoConfig(const std::string& config_file_path);
  bool loadLoggerConfig(const std::string& config_file_path);
  bool initLogger();
  bool setupDecoders();
  void destroyDecoders();
  bool setupPublishers();
  void destroyPublishers();

 private:
  std::string m_video_config_path_ = "";
  std::string m_dds_config_path_ = "";
  std::string m_logger_config_path_ = "";

  std::map<int32_t, VideoConfig_t> m_video_config_;
  LogParam_t m_log_param_;
  int32_t m_logger_ = 0;

  //std::map<int32_t, std::shared_ptr<VideoDecoder>> m_decoders_;
 // 修改为支持多流的解码器映射 [camera_id][stream_index] -> decoder
  std::map<int32_t, std::vector<std::shared_ptr<VideoDecoder>>> m_decoders_;


  std::map<int32_t, std::shared_ptr<DepthImageDecoder>> m_depth_decoders_;

  //std::map<int32_t, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>
      //m_image_publishers_;

  // 修改为支持多流的发布器映射 [camera_id][stream_index] -> publisher
  std::map<int32_t, std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>> m_image_publishers_;



  std::map<int32_t, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>
      m_depth_image_publishers_;

  std::shared_ptr<CDDSWrapper> m_dds_wrapper_;
  std::vector<CDataReader*> m_data_readers_;
};
