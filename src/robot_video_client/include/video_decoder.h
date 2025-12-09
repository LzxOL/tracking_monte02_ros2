/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-16 13:43:16
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-21 10:46:02
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#pragma once

#include <functional>
#include <string>
#include <thread>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include "common_def.h"
#include "data_queue.hpp"
#include "rtsp_client.h"

class VideoDecoder {
 public:
  using DecodeCallback =
      std::function<void(uint8_t* data, uint32_t len, int32_t width,
                         int32_t height, uint32_t seq, int64_t timestamp)>;

  VideoDecoder(int32_t chn_id);
  ~VideoDecoder();

  bool start(const std::string& url, DecodeCallback callback);
  void stop();
  bool pushFrame(Frame_t* frame_ptr);

 private:
  int32_t findVideoStreamIndex(AVFormatContext* fmtCtx);
  void setLowLatencyOptions();
  void decodePacket(AVPacket* pkt);
  void decodePacket(AVPacket* pkt, uint64_t timestamp);
  void doDecode();
  bool reconnect();
  bool openVideoStream();
  void closeVideoStream();
  bool createDecoder();
  void destroyDecoder();
  bool extractTimestamp(uint8_t* data, uint64_t& timestamp);
  void fps();
  void bitrate(uint32_t bits);

 private:
  int32_t m_chn_id_ = -1;
  DecodeCallback m_callback_ = nullptr;
  std::string m_url_ = "";

  AVFormatContext* m_fmtCtx_ = nullptr;
  AVCodecContext* m_decCtx_ = nullptr;
  AVStream* m_videoStream_ = nullptr;
  const AVCodec* m_dec_ = nullptr;
  AVFrame* m_frame_ = nullptr;
  AVBufferRef* m_hwDeviceCtx_ = nullptr;
  SwsContext* m_swsCtx_ = nullptr;

  std::thread m_decodeThread_;
  bool m_running_ = false;

  uint32_t m_frame_count_ = 0;

  int64_t m_last_time_ = 0;
  uint32_t m_count_ = 0;

  uint32_t m_total_bytes_ = 0;
  int64_t m_last_bytes_time_ = 0;

  std::unique_ptr<IRtspClient> m_rtsp_client_ = nullptr;
  std::unique_ptr<DataQueue<Frame_t*>> m_frame_queue_ = nullptr;
};