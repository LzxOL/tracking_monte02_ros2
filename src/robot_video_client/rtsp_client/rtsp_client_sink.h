/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-28 15:29:15
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-21 10:28:15
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#pragma once

#include <BasicUsageEnvironment.hh>
#include <GroupsockHelper.hh>
#include <liveMedia.hh>

#include "common_def.h"

enum NaluType_e {
  NALU_UNDEFINED = 0,
  NALU_SLICE = 1,
  NALU_IDR_SLICE = 5,
  NALU_SEI = 6,
  NALU_SPS = 7,
  NALU_PPS = 8,
  NALU_END
};

class DummySink : public MediaSink {
 public:
  static DummySink* createNew(
      UsageEnvironment& env,
      MediaSubsession&
          subsession,  // identifies the kind of data that's being received
      char const* streamId = NULL);  // identifies the stream itself (optional)

  void setChnID(int32_t chn_id) { m_chn_id_ = chn_id; }

  void registerStreamCallback(StreamDataCallback callback) {
    m_callback_ = callback;
  }

  void registerEventCallback(StreamEventCallback callback) {
    m_event_callback_ = callback;
  }

 private:
  DummySink(UsageEnvironment& env, MediaSubsession& subsession,
            char const* streamId);
  // called only by "createNew()"
  virtual ~DummySink();

  static void afterGettingFrame(void* clientData, unsigned frameSize,
                                unsigned numTruncatedBytes,
                                struct timeval presentationTime,
                                unsigned durationInMicroseconds);
  void afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes,
                         struct timeval presentationTime,
                         unsigned durationInMicroseconds);

 private:
  // redefined virtual functions:
  virtual Boolean continuePlaying();

 private:
  u_int8_t* fReceiveBuffer = nullptr;
  uint32_t fBufferSize = 0;
  uint32_t fDataLen = 0;

  uint8_t* fBuffer = nullptr;

  uint8_t* fSPS = nullptr;
  uint32_t fSPSLen = 0;

  uint8_t* fPPS = nullptr;
  uint32_t fPPSLen = 0;

  uint8_t* fSEI = nullptr;
  uint32_t fSEILen = 0;

  MediaSubsession& fSubsession;
  char* fStreamId;
  int32_t m_chn_id_ = -1;
  StreamDataCallback m_callback_ = nullptr;
  StreamEventCallback m_event_callback_ = nullptr;
};