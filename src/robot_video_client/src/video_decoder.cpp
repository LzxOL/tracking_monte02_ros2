#include "video_decoder.h"

#include "utils.hpp"
#include "yllog.h"

VideoDecoder::VideoDecoder(int32_t chn_id):m_chn_id_(chn_id) {
  std::string tag = "[Video decoder chn " + std::to_string(m_chn_id_) + "]";
  m_frame_queue_ = std::make_unique<DataQueue<Frame_t*>>(
      8,
      [](Frame_t* frame_ptr) {
        if (frame_ptr) free(frame_ptr);
      },
      tag);
}

VideoDecoder::~VideoDecoder() { stop(); }

bool VideoDecoder::reconnect() {
  if (!openVideoStream()) {
    return false;
  }

  if (!createDecoder()) {
    return false;
  }

  YLLOG_DBG("Reconnection successful.");

  return true;
}

void VideoDecoder::fps() {
  m_count_++;

  uint64_t cur_time = getStartupTimeMs();
  if (cur_time - m_last_time_ >= 1000) {
    double fps = m_count_ * 1000.0 / (cur_time - m_last_time_);

    YLLOG_DBG("Video Decoder chn %d: FPS %.1f", m_chn_id_, fps);

    m_count_ = 0;
    m_last_time_ = cur_time;
  }
}

void VideoDecoder::bitrate(uint32_t bytes) {
  m_total_bytes_ += bytes;

  uint64_t cur_time = getStartupTimeMs();
  if (cur_time - m_last_bytes_time_ >= 1000) {
    double bitrate =
        m_total_bytes_ * 8.0 / (getStartupTimeMs() - m_last_bytes_time_);

    YLLOG_DBG("Decoder chn %d: Bitrate %.1f kbps", m_chn_id_, bitrate);

    m_total_bytes_ = 0;
    m_last_bytes_time_ = cur_time;
  }
}

#if 0
bool VideoDecoder::openVideoStream() {
  avformat_network_init();

  if (m_fmtCtx_) {
    avformat_close_input(&m_fmtCtx_);
  }

  AVDictionary* options = nullptr;

  // 设置最小缓存
#if 1
  av_dict_set(&options, "buffer_size", "8192", 0);  // 缓冲区设为 1KB
  av_dict_set(&options, "max_delay", "0", 0);       // 取消最大延迟
  av_dict_set(&options, "rtsp_transport", "tcp", 0);  // 使用 TCP 传输，避免丢包
  av_dict_set(&options, "stimeout", "1000000", 0);  // 超时时间 1 秒（单位：微秒）
  av_dict_set(&options, "tune", "zerolatency", 0);  // 低延迟模式
  av_dict_set(&options, "fflags", "nobuffer", 0);   // 禁用缓冲
  av_dict_set(&options, "flags", "low_delay", 0);   // 低延迟解码
  av_dict_set(&options, "framedrop", "1", 0);  // 允许丢帧，防止累积延迟
  av_dict_set(&options, "probesize", "32", 0);       // 快速解析
  av_dict_set(&options, "analyzeduration", "0", 0);  // 跳过流分析
#else
  // 低延迟优化参数
  av_dict_set(&options, "buffer_size", "20480", 0);  // 缓冲区 10KB
  av_dict_set(&options, "max_delay", "0", 0);        // 取消最大延迟
  av_dict_set(&options, "rtsp_transport", "udp", 0); // 使用 UDP，默认 TCP
  av_dict_set(&options, "stimeout", "1000000", 0);   // 超时时间 1 秒
  av_dict_set(&options, "tune", "zerolatency", 0);   // 低延迟模式
  av_dict_set(&options, "fflags", "nobuffer", 0);    // 禁用缓冲
  av_dict_set(&options, "flags", "low_delay", 0);    // 低延迟解码
  av_dict_set(&options, "framedrop", "1", 0);        // 允许丢帧，防止卡顿
  av_dict_set(&options, "probesize", "8192", 0);     // 解析 8KB，加快初始化
  av_dict_set(&options, "analyzeduration", "50000", 0); // 50ms 内分析完成
#endif

  if (avformat_open_input(&m_fmtCtx_, m_url_.c_str(), nullptr, &options) != 0) {
    std::cerr << "Could not open url '" << m_url_ << "'\n";
    return false;
  }
  av_dict_free(&options);

  if (avformat_find_stream_info(m_fmtCtx_, nullptr) < 0) {
    std::cerr << "Failed to retrieve input stream information\n";
    return false;
  }

  int32_t videoStreamIndex = findVideoStreamIndex(m_fmtCtx_);
  if (videoStreamIndex == -1) {
    std::cerr << "Did not find a video stream\n";
    return false;
  }

  av_dump_format(m_fmtCtx_, 0, m_url_.c_str(), 0);

  m_videoStream_ = m_fmtCtx_->streams[videoStreamIndex];

  return true;
}
#else
bool VideoDecoder::openVideoStream() {
  auto FrameCallback = [this](uint8_t* data, uint32_t len, uint32_t width,
                              uint32_t height, uint64_t timestamp,
                              uint32_t type, uint32_t id) {
    (void)id;

    Frame_t* frame_ptr = (Frame_t*)malloc(sizeof(Frame_t) + len);

    frame_ptr->m_id = -1;
    frame_ptr->m_width = width;
    frame_ptr->m_height = height;
    frame_ptr->m_length = len;
    frame_ptr->m_timestamp = timestamp;
    frame_ptr->m_type = type;
    memcpy(frame_ptr->m_data, data, len);

    pushFrame(frame_ptr);
  };

  m_rtsp_client_ =
      std::make_unique<IRtspClient>(m_url_, FrameCallback, m_chn_id_);
  return m_rtsp_client_->start();
}

#endif
void VideoDecoder::closeVideoStream() {
  if (m_fmtCtx_) {
    avformat_close_input(&m_fmtCtx_);
  }

  if (m_rtsp_client_) {
    m_rtsp_client_->stop();
  }
}

#if 0
bool VideoDecoder::createDecoder() {
  if (!m_videoStream_) {
    std::cerr << "[VideoDecoder] Error: Video stream is null.\n";
    return false;
  }

  m_dec_ = avcodec_find_decoder(m_videoStream_->codecpar->codec_id);
  if (!m_dec_) {
    std::cerr
        << "[VideoDecoder] Error: Failed to find decoder for video stream.\n";
    return false;
  }

  m_decCtx_ = avcodec_alloc_context3(m_dec_);
  if (!m_decCtx_) {
    std::cerr << "[VideoDecoder] Error: Failed to allocate codec context.\n";
    return false;
  }

  if (avcodec_parameters_to_context(m_decCtx_, m_videoStream_->codecpar) < 0) {
    std::cerr << "[VideoDecoder] Error: Failed to copy codec parameters to "
                 "decoder context.\n";
    avcodec_free_context(&m_decCtx_);
    return false;
  }

  // 选择是否启用 CUDA 硬解码
#ifdef USE_CUDA_DECODER
  if (av_hwdevice_ctx_create(&m_hwDeviceCtx_, AV_HWDEVICE_TYPE_CUDA, nullptr,
                             nullptr, 0) < 0) {
    std::cerr << "[VideoDecoder] Error: Failed to create CUDA device.\n";
    avcodec_free_context(&m_decCtx_);
    return false;
  }
  m_decCtx_->hw_device_ctx = av_buffer_ref(m_hwDeviceCtx_);
#endif

  // 设置解码优化参数
  m_decCtx_->flags |= AV_CODEC_FLAG_LOW_DELAY;
  m_decCtx_->flags2 |= AV_CODEC_FLAG2_FAST;
  // m_decCtx_->thread_count = 1;  // 低延迟场景一般使用单线程解码

  if (avcodec_open2(m_decCtx_, m_dec_, nullptr) < 0) {
    std::cerr << "Error: Failed to open codec for decoding.\n";
    avcodec_free_context(&m_decCtx_);
    return false;
  }

  m_frame_ = av_frame_alloc();
  if (!m_frame_) {
    std::cerr << "Error: Failed to allocate video frame.\n";
    avcodec_free_context(&m_decCtx_);
    return false;
  }

  std::cout << "Decoder successfully created.\n";
  return true;
}
#else
bool VideoDecoder::createDecoder() {
  m_dec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (!m_dec_) {
    YLLOG_ERR("Failed to find decoder for video stream.");
    return false;
  }

  m_decCtx_ = avcodec_alloc_context3(m_dec_);
  if (!m_decCtx_) {
    YLLOG_ERR("Failed to allocate codec context.");
    return false;
  }

  // 选择是否启用 CUDA 硬解码
#ifdef USE_CUDA_DECODER
  if (av_hwdevice_ctx_create(&m_hwDeviceCtx_, AV_HWDEVICE_TYPE_CUDA, nullptr,
                             nullptr, 0) < 0) {
    std::cerr << "[VideoDecoder] Error: Failed to create CUDA device.\n";
    avcodec_free_context(&m_decCtx_);
    return false;
  }
  m_decCtx_->hw_device_ctx = av_buffer_ref(m_hwDeviceCtx_);
#endif

  // 设置解码优化参数
  m_decCtx_->flags |= AV_CODEC_FLAG_LOW_DELAY;
  m_decCtx_->flags2 |= AV_CODEC_FLAG2_FAST;
  // m_decCtx_->thread_count = 1;  // 低延迟场景一般使用单线程解码

  if (avcodec_open2(m_decCtx_, m_dec_, nullptr) < 0) {
    YLLOG_ERR("Failed to open codec for decoding.");
    avcodec_free_context(&m_decCtx_);
    return false;
  }

  m_frame_ = av_frame_alloc();
  if (!m_frame_) {
    YLLOG_ERR("Failed to allocate video frame.");
    avcodec_free_context(&m_decCtx_);
    return false;
  }

  YLLOG_INFO("Decoder successfully created.");

  return true;
}
#endif

void VideoDecoder::destroyDecoder() {
  if (m_decCtx_) {
    avcodec_free_context(&m_decCtx_);
    m_decCtx_ = nullptr;
  }

  // av_buffer_unref(&m_hwDeviceCtx_);

  if (m_swsCtx_) {
    sws_freeContext(m_swsCtx_);
    m_swsCtx_ = nullptr;
  }

  if (m_frame_) {
    av_frame_free(&m_frame_);
    m_frame_ = nullptr;
  }
}

#if 0
bool VideoDecoder::start(const std::string& url, DecodeCallback callback) {
  m_callback_ = callback;
  m_url_ = url;

  m_running_ = true;
  m_decodeThread_ = std::thread(&VideoDecoder::doDecode, this);

  return true;
}
#else
bool VideoDecoder::start(const std::string& url, DecodeCallback callback) {
  m_callback_ = callback;
  m_url_ = url;

  if (!openVideoStream()) {
    return false;
  }

  if (!createDecoder()) {
    return false;
  }

  m_running_ = true;
  m_decodeThread_ = std::thread(&VideoDecoder::doDecode, this);

  return true;
}
#endif

void VideoDecoder::stop() {
  m_running_ = false;
  if (m_decodeThread_.joinable()) {
    m_decodeThread_.join();
  }

  closeVideoStream();

  destroyDecoder();
}

bool VideoDecoder::pushFrame(Frame_t* frame_ptr) {
  return m_frame_queue_->enqueue(std::move(frame_ptr));
}

int32_t VideoDecoder::findVideoStreamIndex(AVFormatContext* fmtCtx) {
  for (unsigned i = 0; i < fmtCtx->nb_streams; i++) {
    if (fmtCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
      return i;
    }
  }
  return -1;
}

void VideoDecoder::setLowLatencyOptions() {
  // Set the delay parameter to 0 to reduce buffering
  m_decCtx_->delay = 0;

  // Enable low delay mode if supported by the codec
  m_decCtx_->flags |= AV_CODEC_FLAG_LOW_DELAY;

  // Disable threading to ensure frames are processed sequentially
  m_decCtx_->thread_count = 1;

  // Reduce the number of reference frames to minimize latency
  if (m_decCtx_->codec->capabilities & AV_CODEC_CAP_FRAME_THREADS) {
    m_decCtx_->refs = 1;
  }

  // Set the real-time flag to prioritize speed over quality
  m_decCtx_->flags |= AV_CODEC_FLAG_UNALIGNED;
}

bool VideoDecoder::extractTimestamp(uint8_t* data, uint64_t& timestamp) {
  if (!data) return false;

  timestamp = (static_cast<uint64_t>(data[0]) << 56) |
              (static_cast<uint64_t>(data[1]) << 48) |
              (static_cast<uint64_t>(data[2]) << 40) |
              (static_cast<uint64_t>(data[3]) << 32) |
              (static_cast<uint64_t>(data[4]) << 24) |
              (static_cast<uint64_t>(data[5]) << 16) |
              (static_cast<uint64_t>(data[6]) << 8) |
              (static_cast<uint64_t>(data[7]));

  return true;
}

void VideoDecoder::decodePacket(AVPacket* pkt) {
  // TimePerf t("decode packet with no cuda");

  uint64_t timestamp = 0;
  extractTimestamp(pkt->data + pkt->size - sizeof(uint64_t), timestamp);

  uint32_t travel_time = getCurrentTimeMs() - timestamp;
  YLLOG_DBG("Image chn %d frame travel befor decode time: %u ms", m_chn_id_,
            travel_time);

  // 最后8字节为时间戳，需要去除
  pkt->size -= sizeof(timestamp);

  // bitrate(pkt->size);

  int32_t response = avcodec_send_packet(m_decCtx_, pkt);
  if (response < 0) {
    YLLOG_ERR("Error sending a packet for decoding");
    return;
  }

  while (response >= 0) {
    response = avcodec_receive_frame(m_decCtx_, m_frame_);
    if (response == AVERROR(EAGAIN) || response == AVERROR_EOF)
      break;
    else if (response < 0) {
      YLLOG_ERR("Error during decoding");
      break;
    }

    // Convert the image from its native format to RGB24
    if (!m_swsCtx_) {
      m_swsCtx_ =
          sws_getContext(m_frame_->width, m_frame_->height, m_decCtx_->pix_fmt,
                         m_frame_->width, m_frame_->height, AV_PIX_FMT_RGB24,
                         SWS_BILINEAR, nullptr, nullptr, nullptr);
    }

    uint8_t* buffer;
    int32_t numBytes = av_image_get_buffer_size(
        AV_PIX_FMT_RGB24, m_frame_->width, m_frame_->height, 1);
    buffer = static_cast<uint8_t*>(av_malloc(numBytes * sizeof(uint8_t)));

    uint8_t* dstData[4];
    int32_t dstLinesize[4];
    av_image_fill_arrays(dstData, dstLinesize, buffer, AV_PIX_FMT_RGB24,
                         m_frame_->width, m_frame_->height, 1);

    sws_scale(m_swsCtx_, m_frame_->data, m_frame_->linesize, 0,
              m_frame_->height, dstData, dstLinesize);

    m_frame_count_++;

    if (m_callback_) {
      m_callback_(buffer, numBytes, m_frame_->width, m_frame_->height,
                  m_frame_count_, timestamp);  //: getCurrentTimestampMs()
    }

    av_freep(&buffer);
    av_frame_unref(m_frame_);
  }
}

void VideoDecoder::decodePacket(AVPacket* pkt, uint64_t timestamp) {
  // TimePerf t("decode packet with no cuda");

  // uint32_t travel_time = getCurrentTimeUs() - timestamp;
  // printf("Image chn %d frame travel befor decode time: %u us\n", m_chn_id_,
  // travel_time);

  int32_t response = avcodec_send_packet(m_decCtx_, pkt);
  if (response < 0) {
    YLLOG_ERR("Error sending a packet for decoding");
    return;
  }

  while (response >= 0) {
    response = avcodec_receive_frame(m_decCtx_, m_frame_);
    if (response == AVERROR(EAGAIN) || response == AVERROR_EOF)
      break;
    else if (response < 0) {
      YLLOG_ERR("Error during decoding");
      break;
    }

    // Convert the image from its native format to RGB24
    if (!m_swsCtx_) {
      m_swsCtx_ =
          sws_getContext(m_frame_->width, m_frame_->height, m_decCtx_->pix_fmt,
                         m_frame_->width, m_frame_->height, AV_PIX_FMT_RGB24,
                         SWS_BILINEAR, nullptr, nullptr, nullptr);
    }

    uint8_t* buffer;
    int32_t numBytes = av_image_get_buffer_size(
        AV_PIX_FMT_RGB24, m_frame_->width, m_frame_->height, 1);
    buffer = static_cast<uint8_t*>(av_malloc(numBytes * sizeof(uint8_t)));

    uint8_t* dstData[4];
    int32_t dstLinesize[4];
    av_image_fill_arrays(dstData, dstLinesize, buffer, AV_PIX_FMT_RGB24,
                         m_frame_->width, m_frame_->height, 1);

    sws_scale(m_swsCtx_, m_frame_->data, m_frame_->linesize, 0,
              m_frame_->height, dstData, dstLinesize);

    m_frame_count_++;

    if (m_callback_) {
      m_callback_(buffer, numBytes, m_frame_->width, m_frame_->height,
                  m_frame_count_, timestamp);  //: getCurrentTimestampMs()
    }

    av_freep(&buffer);
    av_frame_unref(m_frame_);
  }
}

#if 0
void VideoDecoder::doDecode() {
  AVPacket pkt;
  int retry_count = 0;
  int64_t retry_delay_ms = 500;

  bool b_connected = false;

  while (m_running_) {
    if (!b_connected) {
      if (reconnect()) {
        b_connected = true;
        retry_delay_ms = 500;
        retry_count = 0;
        continue;
      }

      retry_count++;
      std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
      retry_delay_ms = std::min(retry_delay_ms * 2, int64_t(2000));
    } else {
      int32_t ret = av_read_frame(m_fmtCtx_, &pkt);
      if (ret < 0) {
        if (ret == AVERROR_EOF || ret == AVERROR_EXIT) {
          std::cerr << "Stream ended or exit signal received.\n";
          // break;
        }

        closeVideoStream();
        destroyDecoder();
        b_connected = false;
        continue;
      }

      if (pkt.stream_index == m_videoStream_->index) {
        decodePacket(&pkt);
      }

      av_packet_unref(&pkt);

      // fps();
    }
  }
}
#else
void VideoDecoder::doDecode() {
  AVPacket* packet = av_packet_alloc();

  while (m_running_) {
    Frame_t* frame_ptr = nullptr;

    if (!m_frame_queue_->dequeue(frame_ptr, 100)) {
      continue;
    }

    av_new_packet(packet, frame_ptr->m_length);
    memcpy(packet->data, frame_ptr->m_data, frame_ptr->m_length);

    decodePacket(packet, frame_ptr->m_timestamp);

    av_packet_unref(packet);
    free(frame_ptr);

    fps();
  }

  av_packet_free(&packet);
}

#endif