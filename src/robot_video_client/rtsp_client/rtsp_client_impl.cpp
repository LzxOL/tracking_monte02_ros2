/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-28 13:49:48
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-21 11:24:37
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#include "rtsp_client_impl.h"

#include "rtsp_client_sink.h"
#include "utils.hpp"

static void continueAfterDESCRIBE(RTSPClient* rtspClient, int32_t resultCode,
                                  char* resultString);
static void continueAfterSETUP(RTSPClient* rtspClient, int32_t resultCode,
                               char* resultString);
static void continueAfterPLAY(RTSPClient* rtspClient, int32_t resultCode,
                              char* resultString);

static void setupNextSubsession(RTSPClient* rtspClient);
static void shutdownStream(RTSPClient* rtspClient);

static void subsessionAfterPlaying(void* clientData);
static void subsessionByeHandler(void* clientData, char const* reason);
static void streamTimerHandler(void* clientData);

UsageEnvironment& operator<<(UsageEnvironment& env,
                             const RTSPClient& rtspClient) {
  return env << "[URL:\"" << rtspClient.url() << "\"]: ";
}

// A function that outputs a string that identifies each subsession (for
// debugging output).  Modify this if you wish:
UsageEnvironment& operator<<(UsageEnvironment& env,
                             const MediaSubsession& subsession) {
  return env << subsession.mediumName() << "/" << subsession.codecName();
}

IRtspClientImpl::IRtspClientImpl(const std::string& url,
                                 StreamDataCallback callback, int32_t chn_id)
    :  m_chn_id_(chn_id), m_url_(url), m_callback_(callback) {}

IRtspClientImpl::~IRtspClientImpl() { stop(); }

bool IRtspClientImpl::start() {
  // 启动一个定时器，用于检测流是否断开
  m_timer_task_id_ = CTimerSingleton::getInstance()->add_task(
      [this]() {
        auto cur_time = getCurrentTimeMs();
        int64_t last_time = m_last_frame_time_.load();

        if (!m_b_stream_open_ || (cur_time - last_time > 3000)) {
          m_b_stream_open_ = false;
          if (openStream()) {
            m_running_ = true;
            m_thread_ = std::make_shared<std::thread>(
                &IRtspClientImpl::eventLoop, this);
          }
        }
      },
      2000, -1);

  return true;
}

void IRtspClientImpl::stop() {
  CTimerSingleton::getInstance()->remove_task(m_timer_task_id_);
  m_timer_task_id_ = -1;

  closeStream();
}

void IRtspClientImpl::eventLoop() {
  m_watch_var_ = 0;
  m_env_->taskScheduler().doEventLoop(&m_watch_var_);
}

bool IRtspClientImpl::openStream() {
  closeStream();

  m_scheduler_ = BasicTaskScheduler::createNew();
  m_env_ = BasicUsageEnvironment::createNew(*m_scheduler_);
  m_rtsp_client_ = MediaRtspClient::createNew(*m_env_, m_url_.c_str(), 0,
                                              "RobotVideoClient", 0);
  ((MediaRtspClient*)(m_rtsp_client_))->setChnID(m_chn_id_);
  ((MediaRtspClient*)(m_rtsp_client_))->registerStreamCallback(m_callback_);
  ((MediaRtspClient*)(m_rtsp_client_))
      ->registerEventCallback(
          [this](int32_t event_id, void* event_data, uint32_t id) {
            (void)event_id;
            (void)event_data;
            (void)id;

            auto cur_time = getCurrentTimeMs();
            int64_t expected = m_last_frame_time_.load();
            while (!m_last_frame_time_.compare_exchange_strong(expected,
                                                               cur_time)) {
            }
          });

  m_rtsp_client_->sendDescribeCommand(continueAfterDESCRIBE);

  if (((MediaRtspClient*)m_rtsp_client_)->getErrno() != 0) {
    return false;
  }

  m_last_frame_time_ = getCurrentTimeMs();

  m_b_stream_open_ = true;

  return true;
}

void IRtspClientImpl::closeStream() {
  if (m_running_) {
    m_running_ = false;
    m_watch_var_ = 1;

    if (m_thread_->joinable()) {
      m_thread_->join();
    }
  }

  if (m_rtsp_client_) {
    shutdownStream(m_rtsp_client_);
    m_rtsp_client_ = nullptr;
  }

  if (m_scheduler_) {
    delete m_scheduler_;
    m_scheduler_ = nullptr;
  }

  if (m_env_) {
    m_env_->reclaim();
    m_env_ = nullptr;
  }

  m_b_stream_open_ = false;
}

void continueAfterDESCRIBE(RTSPClient* rtspClient, int32_t resultCode,
                           char* resultString) {
  do {
    UsageEnvironment& env = rtspClient->envir();                   // alias
    StreamClientState& scs = ((MediaRtspClient*)rtspClient)->scs;  // alias

    if (resultCode != 0) {
      env << *rtspClient << "Failed to get a SDP description: " << resultString
          << "\n";
      delete[] resultString;
      ((MediaRtspClient*)rtspClient)->setErrno(-1);
      break;
    }

    char* const sdpDescription = resultString;
    env << *rtspClient << "Got a SDP description:\n" << sdpDescription << "\n";

    // Create a media session object from this SDP description:
    scs.session = MediaSession::createNew(env, sdpDescription);
    delete[] sdpDescription;  // because we don't need it anymore
    if (scs.session == NULL) {
      env << *rtspClient
          << "Failed to create a MediaSession object from the SDP description: "
          << env.getResultMsg() << "\n";
      ((MediaRtspClient*)rtspClient)->setErrno(-1);
      break;
    } else if (!scs.session->hasSubsessions()) {
      env << *rtspClient
          << "This session has no media subsessions (i.e., no \"m=\" lines)\n";
      ((MediaRtspClient*)rtspClient)->setErrno(-1);
      break;
    }

    // Then, create and set up our data source objects for the session.  We do
    // this by iterating over the session's 'subsessions', calling
    // "MediaSubsession::initiate()", and then sending a RTSP "SETUP" command,
    // on each one. (Each 'subsession' will have its own data source.)
    scs.iter = new MediaSubsessionIterator(*scs.session);
    setupNextSubsession(rtspClient);

    // ((MediaRtspClient *)rtspClient)->setErrno(0);

    return;
  } while (0);

  // An unrecoverable error occurred with this stream.
  // shutdownStream(rtspClient);
}

void continueAfterSETUP(RTSPClient* rtspClient, int32_t resultCode,
                        char* resultString) {
  do {
    UsageEnvironment& env = rtspClient->envir();                   // alias
    StreamClientState& scs = ((MediaRtspClient*)rtspClient)->scs;  // alias

    if (resultCode != 0) {
      env << *rtspClient << "Failed to set up the \"" << *scs.subsession
          << "\" subsession: " << resultString << "\n";
      ((MediaRtspClient*)rtspClient)->setErrno(-1);
      break;
    }

    env << *rtspClient << "Set up the \"" << *scs.subsession
        << "\" subsession (";
    if (scs.subsession->rtcpIsMuxed()) {
      env << "client port " << scs.subsession->clientPortNum();
    } else {
      env << "client ports " << scs.subsession->clientPortNum() << "-"
          << scs.subsession->clientPortNum() + 1;
    }
    env << ")\n";

    // Having successfully setup the subsession, create a data sink for it, and
    // call "startPlaying()" on it. (This will prepare the data sink to receive
    // data; the actual flow of data from the client won't start happening until
    // later, after we've sent a RTSP "PLAY" command.)

    scs.subsession->sink =
        DummySink::createNew(env, *scs.subsession, rtspClient->url());
    // perhaps use your own custom "MediaSink" subclass instead
    if (scs.subsession->sink == NULL) {
      env << *rtspClient << "Failed to create a data sink for the \""
          << *scs.subsession << "\" subsession: " << env.getResultMsg() << "\n";
      ((MediaRtspClient*)rtspClient)->setErrno(-1);
      break;
    }

    ((DummySink*)(scs.subsession->sink))
        ->setChnID(((MediaRtspClient*)rtspClient)->getChnID());
    ((DummySink*)(scs.subsession->sink))
        ->registerStreamCallback(
            ((MediaRtspClient*)rtspClient)->getStreamCallback());
    ((DummySink*)(scs.subsession->sink))
        ->registerEventCallback(
            ((MediaRtspClient*)rtspClient)->getEventCallback());

    env << *rtspClient << "Created a data sink for the \"" << *scs.subsession
        << "\" subsession\n";
    scs.subsession->miscPtr =
        rtspClient;  // a hack to let subsession handler functions get the
                     // "RTSPClient" from the subsession
    scs.subsession->sink->startPlaying(*(scs.subsession->readSource()),
                                       subsessionAfterPlaying, scs.subsession);
    // Also set a handler to be called if a RTCP "BYE" arrives for this
    // subsession:
    if (scs.subsession->rtcpInstance() != NULL) {
      scs.subsession->rtcpInstance()->setByeWithReasonHandler(
          subsessionByeHandler, scs.subsession);
    }

    // ((MediaRtspClient *)rtspClient)->setErrno(0);

  } while (0);

  delete[] resultString;

  // Set up the next subsession, if any:
  setupNextSubsession(rtspClient);
}

void continueAfterPLAY(RTSPClient* rtspClient, int32_t resultCode,
                       char* resultString) {
  Boolean success = False;

  do {
    UsageEnvironment& env = rtspClient->envir();                   // alias
    StreamClientState& scs = ((MediaRtspClient*)rtspClient)->scs;  // alias

    if (resultCode != 0) {
      env << *rtspClient << "Failed to start playing session: " << resultString
          << "\n";
      ((MediaRtspClient*)rtspClient)->setErrno(-1);
      break;
    }

    // Set a timer to be handled at the end of the stream's expected duration
    // (if the stream does not already signal its end using a RTCP "BYE").  This
    // is optional.  If, instead, you want to keep the stream active - e.g., so
    // you can later 'seek' back within it and do another RTSP "PLAY" - then you
    // can omit this code. (Alternatively, if you don't want to receive the
    // entire stream, you could set this timer for some shorter value.)
    if (scs.duration > 0) {
      unsigned const delaySlop =
          2;  // number of seconds extra to delay, after the stream's expected
              // duration.  (This is optional.)
      scs.duration += delaySlop;
      unsigned uSecsToDelay = (unsigned)(scs.duration * 1000000);
      scs.streamTimerTask = env.taskScheduler().scheduleDelayedTask(
          uSecsToDelay, (TaskFunc*)streamTimerHandler, rtspClient);
    }

    env << *rtspClient << "Started playing session";
    if (scs.duration > 0) {
      env << " (for up to " << scs.duration << " seconds)";
    }
    env << "...\n";

    success = True;

    // ((MediaRtspClient *)rtspClient)->setErrno(0);
  } while (0);
  delete[] resultString;

  if (!success) {
    // An unrecoverable error occurred with this stream.
    // shutdownStream(rtspClient);
  }
}

#define REQUEST_STREAMING_OVER_TCP True

void setupNextSubsession(RTSPClient* rtspClient) {
  UsageEnvironment& env = rtspClient->envir();                   // alias
  StreamClientState& scs = ((MediaRtspClient*)rtspClient)->scs;  // alias

  scs.subsession = scs.iter->next();
  if (scs.subsession != NULL) {
    if (!scs.subsession->initiate()) {
      env << *rtspClient << "Failed to initiate the \"" << *scs.subsession
          << "\" subsession: " << env.getResultMsg() << "\n";
      setupNextSubsession(
          rtspClient);  // give up on this subsession; go to the next one
    } else {
      env << *rtspClient << "Initiated the \"" << *scs.subsession
          << "\" subsession (";
      if (scs.subsession->rtcpIsMuxed()) {
        env << "client port " << scs.subsession->clientPortNum();
      } else {
        env << "client ports " << scs.subsession->clientPortNum() << "-"
            << scs.subsession->clientPortNum() + 1;
      }
      env << ")\n";

      // Continue setting up this subsession, by sending a RTSP "SETUP" command:
      rtspClient->sendSetupCommand(*scs.subsession, continueAfterSETUP, False,
                                   REQUEST_STREAMING_OVER_TCP);
    }
    return;
  }

  // We've finished setting up all of the subsessions.  Now, send a RTSP "PLAY"
  // command to start the streaming:
  if (scs.session->absStartTime() != NULL) {
    // Special case: The stream is indexed by 'absolute' time, so send an
    // appropriate "PLAY" command:
    rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY,
                                scs.session->absStartTime(),
                                scs.session->absEndTime());
  } else {
    scs.duration = scs.session->playEndTime() - scs.session->playStartTime();
    rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY);
  }
}

void shutdownStream(RTSPClient* rtspClient) {
  UsageEnvironment& env = rtspClient->envir();                   // alias
  StreamClientState& scs = ((MediaRtspClient*)rtspClient)->scs;  // alias

  // First, check whether any subsessions have still to be closed:
  if (scs.session != NULL) {
    Boolean someSubsessionsWereActive = False;
    MediaSubsessionIterator iter(*scs.session);
    MediaSubsession* subsession;

    while ((subsession = iter.next()) != NULL) {
      if (subsession->sink != NULL) {
        Medium::close(subsession->sink);
        subsession->sink = NULL;

        if (subsession->rtcpInstance() != NULL) {
          subsession->rtcpInstance()->setByeHandler(
              NULL, NULL);  // in case the server sends a RTCP "BYE" while
                            // handling "TEARDOWN"
        }

        someSubsessionsWereActive = True;
      }
    }

    if (someSubsessionsWereActive) {
      // Send a RTSP "TEARDOWN" command, to tell the server to shutdown the
      // stream. Don't bother handling the response to the "TEARDOWN".
      rtspClient->sendTeardownCommand(*scs.session, NULL);
    }
  }

  env << *rtspClient << "Closing the stream.\n";
  Medium::close(rtspClient);
  // Note that this will also cause this stream's "StreamClientState" structure
  // to get reclaimed.

  // exit(exitCode);
}

void subsessionAfterPlaying(void* clientData) {
  MediaSubsession* subsession = (MediaSubsession*)clientData;
  // RTSPClient* rtspClient = (RTSPClient*)(subsession->miscPtr);

  // Begin by closing this subsession's stream:
  Medium::close(subsession->sink);
  subsession->sink = NULL;

  // Next, check whether *all* subsessions' streams have now been closed:
  MediaSession& session = subsession->parentSession();
  MediaSubsessionIterator iter(session);
  while ((subsession = iter.next()) != NULL) {
    if (subsession->sink != NULL) return;  // this subsession is still active
  }

  // All subsessions' streams have now been closed, so shutdown the client:
  // shutdownStream(rtspClient);
}

void subsessionByeHandler(void* clientData, char const* reason) {
  MediaSubsession* subsession = (MediaSubsession*)clientData;
  RTSPClient* rtspClient = (RTSPClient*)subsession->miscPtr;
  UsageEnvironment& env = rtspClient->envir();  // alias

  env << *rtspClient << "Received RTCP \"BYE\"";
  if (reason != NULL) {
    env << " (reason:\"" << reason << "\")";
    delete[](char*) reason;
  }
  env << " on \"" << *subsession << "\" subsession\n";

  // Now act as if the subsession had closed:
  subsessionAfterPlaying(subsession);
}

void streamTimerHandler(void* clientData) {
  MediaRtspClient* rtspClient = (MediaRtspClient*)clientData;
  StreamClientState& scs = rtspClient->scs;  // alias

  scs.streamTimerTask = NULL;

  // Shut down the stream:
  // shutdownStream(rtspClient);
}