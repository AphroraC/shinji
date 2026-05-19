#include <shinji/config_server.hpp>

namespace shinji {
void ConfigServer::load(const std::string& config_file) {
  try {
    std::ifstream file(config_file);
    json data = json::parse(file, nullptr, true, true);

    ros.cloud_topic = data["ros"]["cloud_topic"];
    ros.globalmap_frame = data["ros"]["globalmap_frame"];
    ros.lidar_odom_frame = data["ros"]["lidar_odom_frame"];
    ros.lidar_base_frame = data["ros"]["lidar_base_frame"];
    ros.publish_map2odom_tf = data["ros"]["publish_map2odom_tf"];
    ros.tf_listen_timeout = data["ros"]["tf_listen_timeout"];

    common.circular_buffer_capacity = data["common"]["circular_buffer_capacity"];
    common.transformation_needed = data["common"]["transformation_needed"];
    common.globalmap_directory = data["common"]["globalmap_directory"];
    common.globalmap_origin = data["common"]["globalmap_origin"];
    common.globalmap_filtered = data["common"]["globalmap_filtered"];
    common.globalmap_features = data["common"]["globalmap_features"];
    common.centered = data["common"]["centered"];
    common.centroid = Eigen::Vector4d(data["common"]["centroid"][0], data["common"]["centroid"][1], data["common"]["centroid"][2], 1.0);

    logging.enable = data["logging"]["enable"];
    logging.path = data["logging"]["path"];

    fpfh.normal_estimation_radius = data["fpfh"]["normal_estimation_radius"];
    fpfh.search_radius = data["fpfh"]["search_radius"];
    fpfh.num_threads = data["fpfh"]["num_threads"];

    gicp.enable = data["gicp"]["enable"];
    gicp.source_frames = data["gicp"]["source_frames"];
    gicp.voxel_resolution = data["gicp"]["voxel_resolution"];
    gicp.inlier_fraction_threshold = data["gicp"]["inlier_fraction_threshold"];
    gicp.num_threads = data["gicp"]["num_threads"];
    gicp.num_neighbors = data["gicp"]["num_neighbors"];
    gicp.variance = data["gicp"]["variance"];

    teaser.enable = data["teaser"]["enable"];
    teaser.source_frames = data["teaser"]["source_frames"];
    teaser.voxel_resolution = data["teaser"]["voxel_resolution"];
    teaser.max_correspondence_distance = data["teaser"]["max_correspondence_distance"];
    teaser.inlier_fraction_threshold = data["teaser"]["inlier_fraction_threshold"];
    teaser.cross_check = data["teaser"]["cross_check"];
    teaser.tuple_test = data["teaser"]["tuple_test"];
    teaser.tuple_scale = data["teaser"]["tuple_scale"];
    teaser.noise_bound = data["teaser"]["noise_bound"];
    teaser.cbar2 = data["teaser"]["cbar2"];
    teaser.rotation_max_iterations = data["teaser"]["rotation_max_iterations"];
    teaser.rotation_gnc_factor = data["teaser"]["rotation_gnc_factor"];
    teaser.rotation_cost_threshold = data["teaser"]["rotation_cost_threshold"];
    teaser.cost_estimation_method = data["teaser"]["cost_estimation_method"];

    cropbox.enable = data["cropbox"]["enable"];
    cropbox.negative = data["cropbox"]["negative"];
    cropbox.min = Eigen::Vector4f(data["cropbox"]["min_x"], data["cropbox"]["min_y"], data["cropbox"]["min_z"], 1.0);
    cropbox.max = Eigen::Vector4f(data["cropbox"]["max_x"], data["cropbox"]["max_y"], data["cropbox"]["max_z"], 1.0);

    validation();
  } catch (...) {
    throw std::runtime_error("Error parsing configuration file.");
  }
}

void ConfigServer::validation() {
  bool buffer_invalid = common.circular_buffer_capacity <= 0 || teaser.source_frames <= 0 || gicp.source_frames <= 0 ||
                        common.circular_buffer_capacity < std::max(teaser.source_frames, gicp.source_frames);
  if (buffer_invalid) {
    spdlog::error("Invalid configuration for circular buffer.");
    throw std::runtime_error("Invalid configuration.");
  }

  bool method_invalid = teaser.cost_estimation_method != "Voxels" && teaser.cost_estimation_method != "Flann";
  if (method_invalid) {
    spdlog::error("Invalid configuration for TEASER cost estimation method.");
    throw std::runtime_error("Invalid configuration.");
  }

  bool transform_invalid = !common.transformation_needed && cropbox.enable;
  if (transform_invalid) {
    spdlog::error("Cropbox cannot be applied when transformation is not needed.");
    throw std::runtime_error("Invalid configuration.");
  }
  return;
}
}  // namespace shinji