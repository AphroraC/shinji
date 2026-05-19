#pragma once
#include <string>
#include <filesystem>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/ringbuffer_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>

namespace shinji {
namespace {
std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt> ringbuffer_sink;
}

std::shared_ptr<spdlog::logger> get_default_logger() {
  return spdlog::default_logger();
}

void set_default_logger(const std::shared_ptr<spdlog::logger>& logger) {
  spdlog::set_default_logger(logger);
}

std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt> get_ringbuffer_sink(int buffer_size = 128) {
  if (!ringbuffer_sink) {
    ringbuffer_sink = std::make_shared<spdlog::sinks::ringbuffer_sink_mt>(buffer_size);
  }
  return ringbuffer_sink;
}

std::shared_ptr<spdlog::logger> create_module_logger(const std::string& module_name, const std::string& logging_dir = "") {
  std::shared_ptr<spdlog::logger> logger = spdlog::get(module_name);
  if (logger) {
    return logger;
  }

  if (!std::filesystem::exists(logging_dir)) {
    std::filesystem::create_directories(logging_dir);
  }

  logger = spdlog::stdout_color_mt(module_name);
  logger->sinks().push_back(get_ringbuffer_sink());

  if (logging_dir.empty()) {
    return logger;
  }

  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(logging_dir + module_name + ".log", false);
  logger->sinks().push_back(file_sink);

  logger->set_level(get_default_logger()->level());

  return logger;
}

}  // namespace shinji
