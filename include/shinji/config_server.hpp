#pragma once
#include <string>
#include <memory>
#include <fstream>

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

#include "shinji/utility.hpp"

namespace shinji {
using json = nlohmann::json;

struct RosConfig {
  std::string cloud_topic;
  std::string globalmap_frame;
  std::string lidar_odom_frame;
  std::string lidar_base_frame;
  bool publish_map2odom_tf;
  double tf_listen_timeout;
};

struct CommonConfig {
  int circular_buffer_capacity;
  bool transformation_needed;
  std::string globalmap_directory;
  std::string globalmap_origin;
  std::string globalmap_filtered;
  std::string globalmap_features;
  bool centered;
  Eigen::Vector4d centroid;
};

struct LoggingConfig {
  bool enable;
  std::string path;
};

struct FpfhConfig {
  double normal_estimation_radius;
  double search_radius;
  int num_threads;
};

struct GicpConfig {
  bool enable;
  int source_frames;
  double voxel_resolution;
  double inlier_fraction_threshold;
  int num_threads;
  int num_neighbors;
  double variance;
};

struct TeaserConfig {
  bool enable;
  int source_frames;
  double voxel_resolution;
  double max_correspondence_distance;
  double inlier_fraction_threshold;
  bool cross_check;
  bool tuple_test;
  double tuple_scale;
  double noise_bound;
  double cbar2;
  int rotation_max_iterations;
  double rotation_gnc_factor;
  double rotation_cost_threshold;
  std::string cost_estimation_method;
};

struct CropboxConfig {
  bool enable;
  bool negative;
  Eigen::Vector4f min;
  Eigen::Vector4f max;
};

class ConfigServer {
public:
  ConfigServer() = default;
  ~ConfigServer() = default;

  void load(const std::string& config_file);

  RosConfig ros;
  CommonConfig common;
  LoggingConfig logging;
  FpfhConfig fpfh;
  GicpConfig gicp;
  TeaserConfig teaser;
  CropboxConfig cropbox;

private:
  void validation();
};

}  // namespace shinji