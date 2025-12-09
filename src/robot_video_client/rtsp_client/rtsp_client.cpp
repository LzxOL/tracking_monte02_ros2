#include "rtsp_client.h"

#include "rtsp_client_impl.h"

IRtspClient::IRtspClient(const std::string& url, StreamDataCallback callback,
                         int32_t chn_id) {
  m_impl_ = new IRtspClientImpl(url, callback, chn_id);
}

IRtspClient::~IRtspClient() { delete m_impl_; }

bool IRtspClient::start() { return m_impl_->start(); }

void IRtspClient::stop() { m_impl_->stop(); }