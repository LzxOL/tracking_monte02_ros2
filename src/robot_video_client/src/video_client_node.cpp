/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-16 13:05:09
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-30 10:14:19
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */

#include <opencv2/opencv.hpp>
#include "video_client_node.h"
#include "nlohmann/json.hpp"
#include "utils.hpp"

RobotVideoClient::RobotVideoClient() : Node("robot_video_client") {
  this->declare_parameter(
      "config_file",
      "/home/liuhy/robot_workspace/install/robot_video_client/share/robot_video_client/config/video_config.json");

  this->declare_parameter(
      "dds_config_file",
      "/home/liuhy/robot_workspace/install/robot_video_client/share/robot_video_client/config/dds_config.json");

  this->declare_parameter(
    "logger_config_file",
    "/home/liuhy/robot_workspace/install/robot_video_client/share/robot_video_client/config/logger_config.json");
  
      
  m_video_config_path_ = this->get_parameter("config_file").as_string();
  m_dds_config_path_ = this->get_parameter("dds_config_file").as_string();
  m_logger_config_path_ = this->get_parameter("logger_config_file").as_string();

  m_dds_wrapper_ = std::make_shared<CDDSWrapper>(m_dds_config_path_);

  if(!init()) {
    throw std::runtime_error("RobotVideoClient init failed");
  }
}

RobotVideoClient::~RobotVideoClient() { deinit(); }

bool RobotVideoClient::init() {
  if(!initLogger()) {
    return false;
  }

  if (!loadVideoConfig(m_video_config_path_)) {
    return false;
  }

  if (!setupPublishers()) {
    return false;
  }

  if (!setupDecoders()) {
    return false;
  }

  YLLOG_INFO("RobotVideoClient init success.");

  return true;
}

void RobotVideoClient::deinit() {
  destroyDecoders();
  destroyPublishers();
}

/*
bool RobotVideoClient::loadVideoConfig(const std::string &config_file_path) {
  std::ifstream file(config_file_path);
  if (!file.is_open()) {
    YLLOG_ERR("Failed to open JSON file: %s", config_file_path.c_str());
    return false;
  }

  nlohmann::json jsonData;
  file >> jsonData;

  auto videos = jsonData["videos"];
  for (auto &video : videos) {
    VideoConfig_t vidConfig;

    vidConfig.m_id = video["id"].get<int32_t>();
    vidConfig.m_enable = video["enable"].get<bool>();
    vidConfig.m_name = video["name"].get<std::string>();
    vidConfig.m_url = video["url"].get<std::string>();
    vidConfig.m_pub_topic_color_image_raw = video["pub_topic_color_image_raw"].get<std::string>();
    vidConfig.m_sub_topic_depth_image_compressed = video["sub_topic_depth_image_compressed"].get<std::string>();
    vidConfig.m_pub_topic_depth_image_raw = video["pub_topic_depth_image_raw"].get<std::string>();

    m_video_config_[vidConfig.m_id] = vidConfig;
  }

  for (auto &item : m_video_config_) {
    YLLOG_DBG("Camera ID: %d", item.second.m_id);
    YLLOG_DBG("Camera Enable: %d", item.second.m_enable);
    YLLOG_DBG("Camera Name: %s", item.second.m_name.c_str());
    YLLOG_DBG("Camera URL: %s", item.second.m_url.c_str());
    YLLOG_DBG("Camera Color Image Raw Topic: %s", item.second.m_pub_topic_color_image_raw.c_str());
    YLLOG_DBG("Camera Depth Image Compressed Topic: %s", item.second.m_sub_topic_depth_image_compressed.c_str());
    YLLOG_DBG("Camera Depth Image Raw Topic: %s", item.second.m_pub_topic_depth_image_raw.c_str());
  }

  return true;
}*/

bool RobotVideoClient::loadVideoConfig(const std::string &config_file_path) {
  std::ifstream file(config_file_path);
  if (!file.is_open()) {
    YLLOG_ERR("Failed to open JSON file: %s", config_file_path.c_str());
    return false;
  }

  nlohmann::json jsonData;
  file >> jsonData;

  auto videos = jsonData["videos"];
  for (auto &video : videos) {
    VideoConfig_t vidConfig;

    vidConfig.m_id = video["id"].get<int32_t>();
    vidConfig.m_enable = video["enable"].get<bool>();
    vidConfig.m_name = video["name"].get<std::string>();
    
    // 检查是否有streams配置（新格式）
    if (video.contains("streams") && video["streams"].is_array()) {
      // 新格式：支持多流
      auto streams = video["streams"];
      for (auto &stream : streams) {
        StreamConfig_t streamConfig;
        streamConfig.m_url = stream["url"].get<std::string>();
        streamConfig.m_pub_topic_color_image_raw = stream["pub_topic_color_image_raw"].get<std::string>();
        vidConfig.m_streams.push_back(streamConfig);
      }
      YLLOG_INFO("Camera %d (%s) loaded with %zu streams", 
                 vidConfig.m_id, vidConfig.m_name.c_str(), vidConfig.m_streams.size());
    } else {
      // 旧格式：单个流，保持兼容性
      if (video.contains("url") && video.contains("pub_topic_color_image_raw")) {
        StreamConfig_t streamConfig;
        streamConfig.m_url = video["url"].get<std::string>();
        streamConfig.m_pub_topic_color_image_raw = video["pub_topic_color_image_raw"].get<std::string>();
        vidConfig.m_streams.push_back(streamConfig);
        
        // 也保存到原有字段以兼容现有代码
        vidConfig.m_url = streamConfig.m_url;
        vidConfig.m_pub_topic_color_image_raw = streamConfig.m_pub_topic_color_image_raw;
      }
      YLLOG_INFO("Camera %d (%s) loaded with legacy single stream format", 
                 vidConfig.m_id, vidConfig.m_name.c_str());
    }

    // 深度相关配置
    if (video.contains("sub_topic_depth_image_compressed")) {
      vidConfig.m_sub_topic_depth_image_compressed = video["sub_topic_depth_image_compressed"].get<std::string>();
    }
    if (video.contains("pub_topic_depth_image_raw")) {
      vidConfig.m_pub_topic_depth_image_raw = video["pub_topic_depth_image_raw"].get<std::string>();
    }

    m_video_config_[vidConfig.m_id] = vidConfig;
  }

  // 打印配置信息
  for (auto &item : m_video_config_) {
    YLLOG_DBG("Camera ID: %d", item.second.m_id);
    YLLOG_DBG("Camera Enable: %d", item.second.m_enable);
    YLLOG_DBG("Camera Name: %s", item.second.m_name.c_str());
    
    for (size_t i = 0; i < item.second.m_streams.size(); ++i) {
      YLLOG_DBG("  Stream %zu URL: %s", i, item.second.m_streams[i].m_url.c_str());
      YLLOG_DBG("  Stream %zu Topic: %s", i, item.second.m_streams[i].m_pub_topic_color_image_raw.c_str());
    }
    
    if (!item.second.m_sub_topic_depth_image_compressed.empty()) {
      YLLOG_DBG("Camera Depth Image Compressed Topic: %s", item.second.m_sub_topic_depth_image_compressed.c_str());
    }
    if (!item.second.m_pub_topic_depth_image_raw.empty()) {
      YLLOG_DBG("Camera Depth Image Raw Topic: %s", item.second.m_pub_topic_depth_image_raw.c_str());
    }
  }

  return true;
}

bool RobotVideoClient::loadLoggerConfig(const std::string &config_file_path) {
  std::ifstream file(config_file_path);
  if (!file.is_open()) {
    std::cerr << "Failed to open JSON file: " << config_file_path << std::endl;
    return false;
  }

  nlohmann::json jsonData;
  file >> jsonData;

  nlohmann::json logConfig = jsonData["log"];

  nlohmann::json consoleConfig = logConfig["console"];
  bool console_enalbe = consoleConfig["enable"].get<bool>();
  if(console_enalbe) {
    m_logger_ |= LOGGER_CONS;
  }
  m_log_param_.m_stConParam.m_iLevel = consoleConfig["level"].get<int32_t>();
  m_log_param_.m_stConParam.m_iPattern = 0;
  strcpy(m_log_param_.m_stConParam.m_acLoggerName, "consolelogger");

  nlohmann::json fileConfig = logConfig["file"];
  bool file_enable = fileConfig["enable"].get<bool>();
  if(file_enable) {
    m_logger_ |= LOGGER_FILE;
  }
  m_log_param_.m_stFileParam.m_iRollType = fileConfig["rollType"].get<int32_t>();
  m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_iLevel = fileConfig["level"].get<int32_t>();
  m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_bImmediateFlush = fileConfig["immediaFlush"].get<bool>();
  m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_iBufferSize = fileConfig["buffSize"].get<int32_t>();
  std::string file_path = fileConfig["filePath"].get<std::string>();
  if(!file_path.empty()) {
    strcpy(m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_acFilePath, fileConfig["filePath"].get<std::string>().c_str());
  }
  std::string file_name = fileConfig["fileName"].get<std::string>();
  if(!file_name.empty()) {
    strcpy(m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_acFileName, fileConfig["fileName"].get<std::string>().c_str());
  }
  m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_iMaxBackupIndex = fileConfig["MaxBackupIndex"].get<int32_t>();
  m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_iMaxFileSize = fileConfig["MaxFileSize"].get<int32_t>();
  m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_iPattern = 0;
  strcpy(m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_acLoggerName, "filelogger");

  #if 0
  // 打印日志配置
  std::cout << "Logger Config:" << std::endl;
  std::cout << "Console Enable: " << console_enalbe << std::endl;
  std::cout << "Console Level: " << m_log_param_.m_stConParam.m_iLevel << std::endl;
  std::cout << "File Enable: " << file_enable << std::endl;
  std::cout << "File Roll Type: " << m_log_param_.m_stFileParam.m_iRollType << std::endl;
  std::cout << "File Level: " << m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_iLevel << std::endl;
  std::cout << "File Immediate Flush: " << m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_bImmediateFlush << std::endl;
  std::cout << "File Buffer Size: " << m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_iBufferSize << std::endl;
  std::cout << "File Path: " << m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_acFilePath << std::endl;
  std::cout << "File Name: " << m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_acFileName << std::endl;
  std::cout << "File Max Backup Index: " << m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_iMaxBackupIndex << std::endl;
  std::cout << "File Max File Size: " << m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_iMaxFileSize << std::endl;
  std::cout << "File Pattern: " << m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_iPattern << std::endl;
  std::cout << "File Logger Name: " << m_log_param_.m_stFileParam.m_unParam.m_stRollIndexParam.m_acLoggerName << std::endl;
  #endif

  return true;
}

bool RobotVideoClient::initLogger() {
  YLLOG_GetDefaultParam(&m_log_param_);
  if (!loadLoggerConfig(m_logger_config_path_)) {
    return false;
  }
  YLLOG_Init(m_logger_, &m_log_param_);

  return true;
}

#if 0
bool RobotVideoClient::setupDecoders() {
  for (auto &item : m_video_config_) {
    if (!item.second.m_enable) {
      continue;
    }

    std::shared_ptr<VideoDecoder> p_decoder =
        std::make_shared<VideoDecoder>(item.second.m_id);

    auto DataCallback = [this, id = item.second.m_id](uint8_t *data, uint32_t len, int32_t width,
                                                      int32_t height, uint32_t seq,
                                                      uint64_t timestamp) {
      // printf("Received frame from camera: %d, width: %d, height: %d, seq: %d,
      // timestamp: %lu\n", id, width, height, seq, timestamp);

      YLLOG_DBG("Image chn %d frame travel time: %u us", id, getCurrentTimeUs() - timestamp);

      sensor_msgs::msg::Image::UniquePtr image_msg(
          new sensor_msgs::msg::Image());
      image_msg->header.stamp.sec = timestamp / 1000000;
      image_msg->header.stamp.nanosec = (timestamp % 1000000) * 1000;
      image_msg->header.frame_id = seq;
      image_msg->width = width;
      image_msg->height = height;
      image_msg->encoding = "rgb8";
      image_msg->is_bigendian = false;
      image_msg->step = width * 3;
      image_msg->data.resize(len);
      memcpy(image_msg->data.data(), data, len);

      m_image_publishers_[id]->publish(*image_msg);

      // if(id == 2)
      // {
      //     displayRGBFrame(data, width, height);
      // }
    };

    if (p_decoder->start(item.second.m_url, DataCallback)) {
      m_decoders_[item.second.m_id] = p_decoder;
    } else {
      YLLOG_ERR("Failed to start decoder for camera: %s", item.second.m_name.c_str());
    }

    // 添加深度图像解码器
    auto DepthDataCallback = [this, id = item.second.m_id](uint8_t *data, uint32_t len, int32_t width,
      int32_t height, uint32_t seq,
      uint64_t timestamp) {
      (void)seq;

      YLLOG_DBG("Depth chn %d frame travel time: %u us", id, getCurrentTimeUs() - timestamp);

      sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());

      image_msg->header.stamp.sec = timestamp / 1000000;
      image_msg->header.stamp.nanosec = (timestamp % 1000000) * 1000;
      image_msg->header.frame_id = "depth_camera_" + std::to_string(id);

      image_msg->width = width;
      image_msg->height = height;
      image_msg->encoding = "16UC1";
      image_msg->is_bigendian = false;
      image_msg->step = width * sizeof(uint16_t);
      image_msg->data.resize(len);

      memcpy(image_msg->data.data(), data, len);

      m_depth_image_publishers_[id]->publish(*image_msg);

      // if (id == 0) {
      //     displayDepthImage(reinterpret_cast<uint16_t*>(data), width, height);
      // }
      };


    m_depth_decoders_[item.second.m_id] = std::make_shared<DepthImageDecoder>(item.second.m_id);
    m_depth_decoders_[item.second.m_id]->start(m_dds_wrapper_->createDataReader(item.second.m_sub_topic_depth_image_compressed), DepthDataCallback);
  
  }

  return true;
}

void RobotVideoClient::destroyDecoders() {
  for (auto &item : m_decoders_) {
    item.second->stop();
  }
  m_decoders_.clear();

  for (auto &item : m_depth_decoders_) {
    item.second->stop();
  }
  m_depth_decoders_.clear();
}


bool RobotVideoClient::setupPublishers() {
  for (auto &item : m_video_config_) {
    if (!item.second.m_enable) {
      continue;
    }

    #if 1
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));   
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);  
    qos.history(rclcpp::HistoryPolicy::KeepLast);  
    qos.keep_last(2);
    #else
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    #endif

    m_image_publishers_[item.second.m_id] =
        create_publisher<sensor_msgs::msg::Image>(item.second.m_pub_topic_color_image_raw, qos);





    m_depth_image_publishers_[item.second.m_id] =
        create_publisher<sensor_msgs::msg::Image>(item.second.m_pub_topic_depth_image_raw, qos);
  }

  return true;
}

void RobotVideoClient::destroyPublishers() {
  // for (auto &item : m_image_publishers_)
  // {
  //     item.second.reset();
  // }

  m_image_publishers_.clear();
  m_depth_image_publishers_.clear();
}
#endif

bool RobotVideoClient::setupDecoders() {
  for (auto &item : m_video_config_) {
    if (!item.second.m_enable) {
      continue;
    }

    int32_t camera_id = item.second.m_id;
    
    // 为每个摄像头初始化解码器向量
    m_decoders_[camera_id] = std::vector<std::shared_ptr<VideoDecoder>>();
    
    // 为每个流创建解码器
    for (size_t stream_idx = 0; stream_idx < item.second.m_streams.size(); ++stream_idx) {
      const auto& stream = item.second.m_streams[stream_idx];
      
      // 创建唯一的解码器ID（camera_id * 1000 + stream_idx）
      int32_t decoder_id = camera_id * 1000 + static_cast<int32_t>(stream_idx);
      std::shared_ptr<VideoDecoder> p_decoder = std::make_shared<VideoDecoder>(decoder_id);

      auto DataCallback = [this, camera_id, stream_idx](
          uint8_t *data, uint32_t len, int32_t width,
          int32_t height, uint32_t seq, uint64_t timestamp) {
        
        YLLOG_DBG("Image chn %d stream %zu frame travel time: %u us", 
                  camera_id, stream_idx, getCurrentTimeUs() - timestamp);

        sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());
        image_msg->header.stamp.sec = timestamp / 1000000;
        image_msg->header.stamp.nanosec = (timestamp % 1000000) * 1000;
        image_msg->header.frame_id = seq;
        image_msg->width = width;
        image_msg->height = height;
        image_msg->encoding = "rgb8";
        image_msg->is_bigendian = false;
        image_msg->step = width * 3;
        image_msg->data.resize(len);
        memcpy(image_msg->data.data(), data, len);

        // 发布到对应的流发布器
        if (m_image_publishers_.find(camera_id) != m_image_publishers_.end() &&
            stream_idx < m_image_publishers_[camera_id].size()) {
          m_image_publishers_[camera_id][stream_idx]->publish(*image_msg);
        }
      };

      if (p_decoder->start(stream.m_url, DataCallback)) {
        m_decoders_[camera_id].push_back(p_decoder);
        YLLOG_INFO("Started decoder for camera %d stream %zu: %s -> %s", 
                   camera_id, stream_idx, stream.m_url.c_str(), 
                   stream.m_pub_topic_color_image_raw.c_str());
      } else {
        YLLOG_ERR("Failed to start decoder for camera %d stream %zu: %s", 
                  camera_id, stream_idx, stream.m_url.c_str());
      }
    }

    // 添加深度图像解码器（如果配置了深度主题）
    if (!item.second.m_sub_topic_depth_image_compressed.empty()) {
      auto DepthDataCallback = [this, camera_id](
          uint8_t *data, uint32_t len, int32_t width,
          int32_t height, uint32_t seq, uint64_t timestamp) {
        (void)seq;

        YLLOG_DBG("Depth chn %d frame travel time: %u us", 
                  camera_id, getCurrentTimeUs() - timestamp);

        sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());

        image_msg->header.stamp.sec = timestamp / 1000000;
        image_msg->header.stamp.nanosec = (timestamp % 1000000) * 1000;
        image_msg->header.frame_id = "depth_camera_" + std::to_string(camera_id);

        image_msg->width = width;
        image_msg->height = height;
        image_msg->encoding = "16UC1";
        image_msg->is_bigendian = false;
        image_msg->step = width * sizeof(uint16_t);
        image_msg->data.resize(len);

        memcpy(image_msg->data.data(), data, len);

        m_depth_image_publishers_[camera_id]->publish(*image_msg);
      };

      m_depth_decoders_[camera_id] = std::make_shared<DepthImageDecoder>(camera_id);
      m_depth_decoders_[camera_id]->start(
          m_dds_wrapper_->createDataReader(item.second.m_sub_topic_depth_image_compressed), 
          DepthDataCallback);
    }
  }

  return true;
}

void RobotVideoClient::destroyDecoders() {
  for (auto &camera_decoders : m_decoders_) {
    for (auto &decoder : camera_decoders.second) {
      decoder->stop();
    }
  }
  m_decoders_.clear();

  for (auto &item : m_depth_decoders_) {
    item.second->stop();
  }
  m_depth_decoders_.clear();
}


bool RobotVideoClient::setupPublishers() {
  for (auto &item : m_video_config_) {
    if (!item.second.m_enable) {
      continue;
    }

    int32_t camera_id = item.second.m_id;

    #if 1
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));   
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);  
    qos.history(rclcpp::HistoryPolicy::KeepLast);  
    qos.keep_last(2);
    #else
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    #endif

    // 为每个摄像头初始化发布器向量
    m_image_publishers_[camera_id] = std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>();
    
    // 为每个流创建发布器
    for (const auto& stream : item.second.m_streams) {
      auto publisher = create_publisher<sensor_msgs::msg::Image>(stream.m_pub_topic_color_image_raw, qos);
      m_image_publishers_[camera_id].push_back(publisher);
      
      YLLOG_INFO("Created publisher for camera %d topic: %s", 
                 camera_id, stream.m_pub_topic_color_image_raw.c_str());
    }

    // 创建深度图像发布器（如果配置了深度主题）
    if (!item.second.m_pub_topic_depth_image_raw.empty()) {
      m_depth_image_publishers_[camera_id] = 
          create_publisher<sensor_msgs::msg::Image>(item.second.m_pub_topic_depth_image_raw, qos);
      
      YLLOG_INFO("Created depth publisher for camera %d topic: %s", 
                 camera_id, item.second.m_pub_topic_depth_image_raw.c_str());
    }
  }

  return true;
}

void RobotVideoClient::destroyPublishers() {
  m_image_publishers_.clear();
  m_depth_image_publishers_.clear();
}
