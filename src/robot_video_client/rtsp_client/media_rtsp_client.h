/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-28 14:47:45
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-21 10:27:12
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#pragma once

#include <BasicUsageEnvironment.hh>
#include <GroupsockHelper.hh>
#include <liveMedia.hh>

#include "common_def.h"

class StreamClientState {
 public:
  StreamClientState();
  virtual ~StreamClientState();

 public:
  MediaSubsessionIterator* iter = nullptr;
  MediaSession* session = nullptr;
  MediaSubsession* subsession = nullptr;
  TaskToken streamTimerTask;
  double duration = 0.0;
};

class MediaRtspClient : public RTSPClient {
 public:
  static MediaRtspClient* createNew(UsageEnvironment& env, char const* rtspURL,
                                    int verbosityLevel = 0,
                                    char const* applicationName = NULL,
                                    portNumBits tunnelOverHTTPPortNum = 0);

  void setChnID(int32_t chn_id) { m_chn_id_ = chn_id; }
  int32_t getChnID() { return m_chn_id_; }
  void setErrno(int32_t err_no) { m_errno_ = err_no; }
  int32_t getErrno() { return m_errno_; }
  void registerStreamCallback(StreamDataCallback callback) {
    m_callback_ = callback;
  }
  StreamDataCallback getStreamCallback() const { return m_callback_; }
  void registerEventCallback(StreamEventCallback callback) {
    m_event_callback_ = callback;
  }
  StreamEventCallback getEventCallback() const { return m_event_callback_; }

 protected:
  MediaRtspClient(UsageEnvironment& env, char const* rtspURL,
                  int verbosityLevel, char const* applicationName,
                  portNumBits tunnelOverHTTPPortNum);
  // called only by createNew();
  virtual ~MediaRtspClient();

 public:
  StreamClientState scs;
  int32_t m_chn_id_ = -1;
  int32_t m_errno_ = 0;
  StreamDataCallback m_callback_ = nullptr;
  StreamEventCallback m_event_callback_ = nullptr;
};
