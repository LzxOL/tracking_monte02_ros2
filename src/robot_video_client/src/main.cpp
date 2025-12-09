/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-16 13:43:16
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-21 14:29:48
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#include <execinfo.h>

#include <csignal>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "video_client_node.h"

std::string g_log_prefix = "robot_video_client";

void logStackTrace(int sig) {
  const int max_frames = 32;
  void* buffer[max_frames];

  int frame_count = backtrace(buffer, max_frames);
  char** symbols = backtrace_symbols(buffer, frame_count);

  std::string log_dir = "Log/";
  if (!std::filesystem::exists(log_dir)) {
    std::filesystem::create_directories(log_dir);
  }

  std::time_t now = std::time(nullptr);
  std::tm* local_time = std::localtime(&now);
  std::ostringstream time_stream;
  time_stream << std::put_time(local_time, "%Y_%m_%d_%H_%M_%S");

  std::string log_file_name =
      g_log_prefix + "_crash_stack_trace_" + time_stream.str() + ".log";
  std::string log_file_path = log_dir + log_file_name;

  std::ofstream log_file(log_file_path, std::ios::app);
  if (log_file.is_open()) {
    log_file << "Received signal: " << sig << "\n";
    log_file << "Stack trace:\n";

    for (int i = 0; i < frame_count; i++) {
      std::cerr << symbols[i] << std::endl;
      log_file << symbols[i] << "\n";
    }

    log_file.close();
  }

  if (symbols) {
    free(symbols);
  }
}

void signalHandler(int sig) {
  std::cout << "Received signal: " << sig << std::endl;
  logStackTrace(sig);
  exit(sig);
}

int32_t main(int32_t argc, char* argv[]) {
  signal(SIGSEGV, signalHandler);  // segment fault
  signal(SIGABRT, signalHandler);  // abort
  signal(SIGFPE, signalHandler);   // float point exception
  signal(SIGILL, signalHandler);   // illegal instruction
  signal(SIGPIPE, SIG_IGN);
  signal(SIGCHLD, SIG_IGN);
  
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  rclcpp::spin(std::make_shared<RobotVideoClient>());
  rclcpp::shutdown();

  return 0;
}