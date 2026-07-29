#pragma once
#include <string>
#include <algorithm>
#include <cctype>
#include <filesystem>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/ringbuffer_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <shinji/config_server.hpp>

namespace shinji {

struct LoggingConfig;

spdlog::level::level_enum log_level_from_string(const std::string& level);

void configure_logging(const LoggingConfig& cfg);

std::shared_ptr<spdlog::logger> create_module_logger(const std::string& module_name, const LoggingConfig& cfg);

}  // namespace shinji
