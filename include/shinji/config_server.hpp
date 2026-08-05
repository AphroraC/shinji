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
  std::string cloud_topic{"/cloud_registered"};
  std::string globalmap_frame{"map"};
  std::string lidar_odom_frame{"camera_init"};
  std::string lidar_base_frame{"body"};
  bool publish_map2odom_tf{false};
  double tf_listen_timeout{0.1};
};

struct CommonConfig {
  int circular_buffer_capacity{10};
  bool transformation_needed{false};
  std::string globalmap_directory{""};
  std::string globalmap_origin{"globalmap_origin.pcd"};
  std::string globalmap_filtered{"globalmap_filtered.pcd"};
  std::string globalmap_features{"globalmap_features.pcd"};
  bool centered{false};
  Eigen::Vector4d centroid{Eigen::Vector4d::Zero()};
};

struct LoggingConfig {
  bool enable{true};
  std::string pcd_saving_path{""};
  bool console_output{true};
  std::string console_level{"info"};
  bool file_output{true};
  std::string logging_dir{""};
  std::string logging_level{"info"};
  std::string flush_level{"info"};
  bool rotate_logs{false};
  size_t max_file_size_kb{8192};
  size_t max_files{10};
};

struct FpfhConfig {
  double normal_estimation_radius{1.0};
  double search_radius{2.0};
  int num_threads{10};
};

struct GicpConfig {
  bool enable{true};
  int source_frames{6};
  double voxel_resolution{0.1};
  double inlier_fraction_threshold{0.8};
  int num_threads{12};
  int num_neighbors{15};
  double variance{0.25};
};

struct TeaserConfig {
  bool enable{true};
  int source_frames{6};
  double voxel_resolution{0.8};
  double max_correspondence_distance{0.8};
  double inlier_fraction_threshold{0.5};
  bool cross_check{true};
  bool tuple_test{false};
  double tuple_scale{0.95};
  double noise_bound{0.5};
  double cbar2{2.0};
  int rotation_max_iterations{100};
  double rotation_gnc_factor{1.2};
  double rotation_cost_threshold{0.005};
  std::string cost_estimation_method{"Flann"};
};

struct CropboxConfig {
  bool enable{false};
  bool negative{false};
  Eigen::Vector4f min{Eigen::Vector4f::Zero()};
  Eigen::Vector4f max{Eigen::Vector4f::Zero()};
};

struct InitialGuessConfig {
  bool enable{true};
  double voxel_resolution{0.8};
  double inlier_fraction_threshold{0.8};
  Eigen::Vector4f translation{Eigen::Vector4f::Zero()};
  Eigen::Vector4f rotation{Eigen::Vector4f::Zero()};
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
  InitialGuessConfig initial_guess;

private:
  void validation();
};

}  // namespace shinji
