#include <shinji/logging.hpp>
namespace shinji {

namespace {
std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt> ringbuffer_sink;
}  // namespace

static std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt> get_ringbuffer_sink(int buffer_size = 128) {
  if (!ringbuffer_sink) {
    ringbuffer_sink = std::make_shared<spdlog::sinks::ringbuffer_sink_mt>(buffer_size);
  }
  return ringbuffer_sink;
}

spdlog::level::level_enum log_level_from_string(const std::string& level) {
  std::string normalized = level;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (normalized == "trace") return spdlog::level::trace;
  if (normalized == "debug") return spdlog::level::debug;
  if (normalized == "info") return spdlog::level::info;
  if (normalized == "warn" || normalized == "warning") return spdlog::level::warn;
  if (normalized == "error" || normalized == "err") return spdlog::level::err;
  if (normalized == "critical") return spdlog::level::critical;
  if (normalized == "off") return spdlog::level::off;
  return spdlog::level::info;
}

void configure_logging(const LoggingConfig& cfg) {
  const auto level = log_level_from_string(cfg.logging_level);
  const auto flush_level = log_level_from_string(cfg.flush_level);
  spdlog::set_level(level);
  if (spdlog::default_logger()) {
    spdlog::default_logger()->set_level(level);
  }
  spdlog::apply_all([level](const std::shared_ptr<spdlog::logger>& logger) { logger->set_level(level); });

  if (!cfg.console_output && spdlog::default_logger()) {
    auto& sinks = spdlog::default_logger()->sinks();
    sinks.erase(
      std::remove_if(
        sinks.begin(),
        sinks.end(),
        [](const auto& sink) {
          return dynamic_cast<spdlog::sinks::stdout_color_sink_mt*>(sink.get()) != nullptr || dynamic_cast<spdlog::sinks::stdout_color_sink_st*>(sink.get()) != nullptr;
        }),
      sinks.end());
  }

  if (cfg.file_output && !cfg.logging_dir.empty()) {
    if (!std::filesystem::exists(cfg.logging_dir)) {
      std::filesystem::create_directories(cfg.logging_dir);
    }
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(cfg.logging_dir + "/default_logger.log", true);
    file_sink->set_level(level);
    if (!spdlog::default_logger()) {
      spdlog::set_default_logger(std::make_shared<spdlog::logger>("default", spdlog::sinks_init_list{file_sink}));
    } else {
      spdlog::default_logger()->sinks().push_back(file_sink);
      if (file_sink->level() < spdlog::default_logger()->level()) {
        spdlog::default_logger()->set_level(file_sink->level());
      }
    }
    spdlog::default_logger()->flush_on(flush_level);
  }
}

std::shared_ptr<spdlog::logger> create_module_logger(const std::string& module_name,
                                                   const LoggingConfig& cfg) {
  std::shared_ptr<spdlog::logger> logger = spdlog::get(module_name);
  if (logger) {
    return logger;
  }

  if (!cfg.logging_dir.empty() && !std::filesystem::exists(cfg.logging_dir)) {
    std::filesystem::create_directories(cfg.logging_dir);
  }

  const auto logging_level = log_level_from_string(cfg.logging_level);
  const auto console_level = log_level_from_string(cfg.console_level);
  const auto flush_level = log_level_from_string(cfg.flush_level);

  logger = std::make_shared<spdlog::logger>(module_name);
  spdlog::register_logger(logger);

  auto ring_sink = get_ringbuffer_sink();
  ring_sink->set_level(logging_level);
  logger->sinks().push_back(ring_sink);

  if (cfg.console_output) {
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(console_level);
    logger->sinks().push_back(console_sink);
  }

  if (cfg.file_output && !cfg.logging_dir.empty()) {
    std::shared_ptr<spdlog::sinks::sink> file_sink;
    const std::string filepath = cfg.logging_dir + "/" + module_name + "_logger.log";
    if (cfg.rotate_logs) {
      file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(filepath, cfg.max_file_size_kb * 1024, cfg.max_files);
    } else {
      file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(filepath, true);
    }
    file_sink->set_level(logging_level);
    logger->sinks().push_back(file_sink);
  }

  auto min_level = logging_level;
  if (cfg.console_output && console_level < min_level) {
    min_level = console_level;
  }
  logger->set_level(min_level);
  logger->flush_on(flush_level);

  return logger;
}

}  // namespace shinji
