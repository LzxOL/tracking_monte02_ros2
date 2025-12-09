/*
 * @Description: 
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-24 21:18:12
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-21 10:25:03
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#pragma once

#include <iostream>
#include <string>
#include <functional>

#include "yhdds.h"
#include "common_def.h"
#include "zdepth.hpp"

using namespace zdepth;

class DepthImageDecoder {
public:
  using DecodeCallback =
      std::function<void(uint8_t* data, uint32_t len, int32_t width, int32_t height,
                        uint32_t seq, int64_t timestamp)>;

  DepthImageDecoder(int32_t chn_id);
  ~DepthImageDecoder();

  bool start(CDataReader *data_reader, DecodeCallback callback);
  void stop();

private:
  void fps();
  bool decompress(std::vector<uint8_t>& compressed, int32_t& width, int32_t& height, std::vector<uint16_t>& depth);
  bool doDecode(Image_t* image_ptr);
  void decodeThread();

private:
  int32_t m_chn_id_;
  DecodeCallback m_callback_;

  CDataReader* m_data_reader_;

  uint32_t m_count_ = 0;
  uint64_t m_last_time_ = 0;
  uint32_t m_last_frame_id = 0;

  std::atomic_bool m_bRunning{false};
  std::shared_ptr<std::thread> m_decode_thread_;

  DepthCompressor m_depth_decompressor_;
};