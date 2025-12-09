/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-28 15:29:28
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-22 17:38:30
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#include "rtsp_client_sink.h"

#include "utils.hpp"

#define DUMMY_SINK_RECEIVE_BUFFER_SIZE (2 * 1024 * 1024)

DummySink* DummySink::createNew(UsageEnvironment& env,
                                MediaSubsession& subsession,
                                char const* streamId) {
  return new DummySink(env, subsession, streamId);
}

DummySink::DummySink(UsageEnvironment& env, MediaSubsession& subsession,
                     char const* streamId)
    : MediaSink(env), fSubsession(subsession) {
  fStreamId = strDup(streamId);

  fBufferSize = DUMMY_SINK_RECEIVE_BUFFER_SIZE;
  fReceiveBuffer = new u_int8_t[fBufferSize];
  fDataLen = 0;

  fBuffer = new uint8_t[fBufferSize];

  fSPS = new uint8_t[2048];
  fSPSLen = 0;

  fPPS = new uint8_t[2048];
  fPPSLen = 0;

  fSEI = new uint8_t[2048];
  fSEILen = 0;
}

DummySink::~DummySink() {
  delete[] fReceiveBuffer;
  delete[] fStreamId;
  delete[] fBuffer;
  delete[] fSPS;
  delete[] fPPS;
  delete[] fSEI;
}

void DummySink::afterGettingFrame(void* clientData, unsigned frameSize,
                                  unsigned numTruncatedBytes,
                                  struct timeval presentationTime,
                                  unsigned durationInMicroseconds) {
  DummySink* sink = (DummySink*)clientData;

  // 获取媒体子会话类型
  // const char* mediaType = sink->fSubsession.mediumName();
  // if (strcmp(mediaType, "video") == 0) {
  //     printf("Received video frame of size %u bytes\n", frameSize);
  // } else if (strcmp(mediaType, "audio") == 0) {
  //     printf("Received audio frame of size %u bytes\n", frameSize);
  // } else {
  //     printf("Received unknown media type (%s) frame of size %u bytes\n",
  //     mediaType, frameSize);
  // }

  // // 获取 RTP 负载类型
  // printf("RTP Payload Type: %d\n", sink->fSubsession.rtpPayloadFormat());

  sink->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime,
                          durationInMicroseconds);
}

bool extractTimestamp(uint8_t* data, uint64_t& timestamp) {
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

// If you don't want to see debugging output for each received frame, then
// comment out the following line: #define DEBUG_PRINT_EACH_RECEIVED_FRAME 1

void DummySink::afterGettingFrame(unsigned frameSize,
                                  unsigned numTruncatedBytes,
                                  struct timeval presentationTime,
                                  unsigned /*durationInMicroseconds*/) {
// We've just received a frame of data.  (Optionally) print out information
// about it:
#ifdef DEBUG_PRINT_EACH_RECEIVED_FRAME
  if (fStreamId != NULL) envir() << "Stream \"" << fStreamId << "\"; ";
  envir() << fSubsession.mediumName() << "/" << fSubsession.codecName()
          << ":\tReceived " << frameSize << " bytes";
  if (numTruncatedBytes > 0)
    envir() << " (with " << numTruncatedBytes << " bytes truncated)";
  char uSecsStr[6 + 1];  // used to output the 'microseconds' part of the
                         // presentation time
  sprintf(uSecsStr, "%06u", (unsigned)presentationTime.tv_usec);
  envir() << ".\tPresentation time: " << (int)presentationTime.tv_sec << "."
          << uSecsStr;
  if (fSubsession.rtpSource() != NULL &&
      !fSubsession.rtpSource()->hasBeenSynchronizedUsingRTCP()) {
    envir() << "!";  // mark the debugging output to indicate that this
                     // presentation time is not RTCP-synchronized
  }
#ifdef DEBUG_PRINT_NPT
  envir() << "\tNPT: " << fSubsession.getNormalPlayTime(presentationTime);
#endif
  envir() << "\n";
#endif

#if 1

  int8_t start_code[] = {0x00, 0x00, 0x00, 0x01};

  uint8_t nal_type = fReceiveBuffer[0] & 0x1F;  // 获取 NAL 类型
  // printf("=================NAL Type: %d\n", nal_type);

  // 数据前添加4字节的起始码
  memcpy(fBuffer + fDataLen, start_code, sizeof(start_code));
  fDataLen += sizeof(start_code);

  memcpy(fBuffer + fDataLen, fReceiveBuffer, frameSize);
  fDataLen += frameSize;

  if (nal_type == 0x05 || nal_type == 0x01) {
    uint64_t timestamp = 0;
    extractTimestamp(fBuffer + fDataLen - sizeof(uint64_t), timestamp);
    // printf("=================timestamp: %lu\n", timestamp);

    // auto cur_time = getCurrentTimeUs();
    // printf("Chn %d: frame travel time %u us\n", m_chn_id_, cur_time -
    // timestamp);

    if (m_callback_) {
      // 最后8个字节是时间戳，去掉
      m_callback_(fBuffer, fDataLen - sizeof(uint64_t), 1280, 720, timestamp, 0,
                  m_chn_id_);
    }

    fDataLen = 0;
  }

  if (m_event_callback_) {
    m_event_callback_(0, nullptr, m_chn_id_);
  }

#else

  int8_t start_code[] = {0x00, 0x00, 0x00, 0x01};

  uint8_t nal_type = fReceiveBuffer[0] & 0x1F;  // 获取 NAL 类型
  // printf("=================NAL Type: %d\n", nal_type);

  if (nal_type == NALU_SEI) {
    // 数据前添加4字节的起始码
    memcpy(fSEI + fSEILen, start_code, sizeof(start_code));
    fSEILen += sizeof(start_code);

    memcpy(fSEI + fSEILen, fReceiveBuffer, frameSize);
    fSEILen += frameSize;
  } else if (nal_type == NALU_SPS) {
    // 数据前添加4字节的起始码
    memcpy(fSPS + fSPSLen, start_code, sizeof(start_code));
    fSPSLen += sizeof(start_code);

    memcpy(fSPS + fSPSLen, fReceiveBuffer, frameSize);
    fSPSLen += frameSize;
  } else if (nal_type == NALU_PPS) {
    // 数据前添加4字节的起始码
    memcpy(fPPS + fPPSLen, start_code, sizeof(start_code));
    fPPSLen += sizeof(start_code);

    memcpy(fPPS + fPPSLen, fReceiveBuffer, frameSize);
    fPPSLen += frameSize;
  } else if (nal_type == NALU_SLICE || nal_type == NALU_IDR_SLICE) {
    uint64_t timestamp = 0;
    extractTimestamp(fReceiveBuffer + frameSize - sizeof(uint64_t), timestamp);

    if (m_callback_) {
      if (fSEILen > 0) {
        m_callback_(fSEI, fSEILen, 1280, 720, timestamp, 0, m_chn_id_);
        // printf("11111 SEI len %u\n", fSEILen);
        fSEILen = 0;
      }

      if (fSPSLen > 0) {
        m_callback_(fSPS, fSPSLen, 1280, 720, timestamp, 0, m_chn_id_);
        // printf("22222 SPS len %u\n", fSPSLen);
        fSPSLen = 0;
      }

      if (fPPSLen > 0) {
        m_callback_(fPPS, fPPSLen, 1280, 720, timestamp, 0, m_chn_id_);
        // printf("33333 PPS len %u\n", fPPSLen);
        fPPSLen = 0;
      }

      memcpy(fBuffer, start_code, sizeof(start_code));
      memcpy(fBuffer + 4, fReceiveBuffer, frameSize);
      fDataLen = 4 + frameSize;

      // 最后8个字节是时间戳，去掉
      m_callback_(fBuffer, fDataLen - sizeof(uint64_t), 1280, 720, timestamp, 0,
                  m_chn_id_);

      // if(nal_type == NALU_SLICE)
      // {
      //   printf("44444 SLICE len %u\n", fDataLen - sizeof(uint64_t));
      // }
      // else
      // {
      //   printf("55555 IDR len %u\n", fDataLen - sizeof(uint64_t));
      // }

      fDataLen = 0;
    }
  }

  if (m_event_callback_) {
    m_event_callback_(0, nullptr, m_chn_id_);
  }

#endif

  // Then continue, to request the next frame of data:
  continuePlaying();
}

Boolean DummySink::continuePlaying() {
  if (fSource == NULL) return False;  // sanity check (should not happen)

  // Request the next frame of data from our input source. "afterGettingFrame()"
  // will get called later, when it arrives:
  fSource->getNextFrame(fReceiveBuffer, fBufferSize, afterGettingFrame, this,
                        onSourceClosure, this);
  return True;
}