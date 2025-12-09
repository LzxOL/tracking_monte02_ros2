/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-25 17:30:49
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-21 10:43:36
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#include "depth_decoder.h"

#include "utils.hpp"
#include "yllog.h"

DepthImageDecoder::DepthImageDecoder(int32_t chn_id) : m_chn_id_(chn_id) {}

DepthImageDecoder::~DepthImageDecoder() { stop(); }

bool DepthImageDecoder::start(CDataReader* data_reader,
                              DecodeCallback callback) {
  if (!data_reader || !callback) {
    return false;
  }

  m_data_reader_ = data_reader;
  m_callback_ = callback;

  m_bRunning = true;
  m_decode_thread_ =
      std::make_shared<std::thread>(&DepthImageDecoder::decodeThread, this);

  return true;
}

void DepthImageDecoder::stop() {
  if (m_bRunning) {
    m_bRunning = false;
    if (m_decode_thread_ && m_decode_thread_->joinable()) {
      m_decode_thread_->join();
    }
  }
}

void DepthImageDecoder::fps() {
  m_count_++;

  uint64_t cur_time = getStartupTimeMs();
  if (cur_time - m_last_time_ >= 1000) {
    double fps = m_count_ * 1000.0 / (cur_time - m_last_time_);
    YLLOG_DBG("Depth Decoder chn %d: FPS %.1f", m_chn_id_, fps);

    m_count_ = 0;
    m_last_time_ = cur_time;
  }
}

bool DepthImageDecoder::decompress(std::vector<uint8_t>& compressed,
                                   int32_t& width, int32_t& height,
                                   std::vector<uint16_t>& depth) {
  if (compressed.size() == 0) {
    return false;
  }

  // uint64_t start = getStartupTimeMs();

  m_depth_decompressor_.Decompress(compressed, width, height, depth);

  // uint64_t end = getStartupTimeMs();

  // printf("depth chn %d: Decompress depth image: %lu ms, width: %d, height:
  // %d\n", m_chn_id_, end - start, width, height);

  return true;
}

bool DepthImageDecoder::doDecode(Image_t* image_ptr) {
  if (!m_callback_) {
    return false;
  }

  if ((m_last_frame_id != 0) && (m_last_frame_id + 1) != image_ptr->m_id) {
    YLLOG_WARN(
        "Depth chn %d: frame id not match, last frame id = %d, current frame "
        "id = %d",
        m_chn_id_, m_last_frame_id, image_ptr->m_id);
  }
  m_last_frame_id = image_ptr->m_id;

  //打印信息
  // std::cout << "DepthImageDecoder::doDecode: id = " << image_ptr->m_id << ",
  // width = " << image_ptr->m_width << ", height = " << image_ptr->m_height <<
  // ", timestamp = " << image_ptr->m_timestamp << std::endl;

  std::vector<uint8_t> compressed(image_ptr->m_data,
                                  image_ptr->m_data + image_ptr->m_length);
  std::vector<uint16_t> m_depth_data_;
  int32_t actual_width = 0;
  int32_t actual_height = 0;
  decompress(compressed, actual_width, actual_height, m_depth_data_);

  if (actual_width != static_cast<int32_t>(image_ptr->m_width) ||
      actual_height != static_cast<int32_t>(image_ptr->m_height)) {
    YLLOG_WARN(
        "Depth chn %d: decompre/ Add this line to include imgutils.hss depth "
        "image failed, actual width = %d, "
        "actual height = %d, expected width = %d, expected height = %d",
        m_chn_id_, actual_width, actual_height, image_ptr->m_width,
        image_ptr->m_height);
  }

  m_callback_((uint8_t*)(m_depth_data_.data()),
              m_depth_data_.size() * sizeof(uint16_t), actual_width,
              actual_height, image_ptr->m_id, image_ptr->m_timestamp);

  fps();

  return true;
}

void DepthImageDecoder::decodeThread() {
  while (m_bRunning) {
    DataBufferPtr pkg = m_data_reader_->getDataBuffer(100);
    if (!pkg) {
      continue;
    }

    doDecode(reinterpret_cast<Image_t*>(pkg->m_pBufPtr));

    m_data_reader_->releaseDataBuffer(pkg);
  }
}