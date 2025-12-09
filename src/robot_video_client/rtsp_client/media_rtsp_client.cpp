/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-28 14:55:28
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-21 10:27:05
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#include "media_rtsp_client.h"

StreamClientState::StreamClientState()
    : iter(NULL),
      session(NULL),
      subsession(NULL),
      streamTimerTask(NULL),
      duration(0.0) {}

StreamClientState::~StreamClientState() {
  delete iter;
  if (session != NULL) {
    // We also need to delete "session", and unschedule "streamTimerTask" (if
    // set)
    UsageEnvironment& env = session->envir();  // alias

    env.taskScheduler().unscheduleDelayedTask(streamTimerTask);
    Medium::close(session);
  }
}

MediaRtspClient* MediaRtspClient::createNew(UsageEnvironment& env,
                                            char const* rtspURL,
                                            int verbosityLevel,
                                            char const* applicationName,
                                            portNumBits tunnelOverHTTPPortNum) {
  return new MediaRtspClient(env, rtspURL, verbosityLevel, applicationName,
                             tunnelOverHTTPPortNum);
}

MediaRtspClient::MediaRtspClient(UsageEnvironment& env, char const* rtspURL,
                                 int verbosityLevel,
                                 char const* applicationName,
                                 portNumBits tunnelOverHTTPPortNum)
    : RTSPClient(env, rtspURL, verbosityLevel, applicationName,
                 tunnelOverHTTPPortNum, -1) {}

MediaRtspClient::~MediaRtspClient() {}
