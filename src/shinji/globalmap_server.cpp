#include <shinji/globalmap_server.hpp>

namespace shinji {
GlobalmapServer::GlobalmapServer(const std::string& directory) : directory(directory) {
  if (this->directory.back() != '/') {
    this->directory += '/';
    spdlog::warn("Directory should end with '/'. Automatically appended.");
  }
}

void GlobalmapServer::load(const std::string& filename, const std::string& type) {
  if (type == "origin") {
    pointcloud_origin.reset(new pcl::PointCloud<PointT>());
    if (pcl::io::loadPCDFile(directory + filename, *pointcloud_origin) == -1) {
      spdlog::error("Could not read file: {}", directory + filename);
    } else {
      spdlog::info("Globalmap Origin Points: {}", pointcloud_origin->size());
    }
  } else if (type == "filtered") {
    pointcloud_filtered.reset(new pcl::PointCloud<PointT>());
    if (pcl::io::loadPCDFile(directory + filename, *pointcloud_filtered) == -1) {
      spdlog::error("Could not read file: {}", directory + filename);
    } else {
      spdlog::info("Globalmap Filtered Points: {}", pointcloud_filtered->size());
    }
  } else if (type == "features") {
    pointcloud_features.reset(new pcl::PointCloud<FeatureT>());
    if (pcl::io::loadPCDFile(directory + filename, *pointcloud_features) == -1) {
      spdlog::error("Could not read file: {}", directory + filename);
    } else {
      spdlog::info("Globalmap Features Points: {}", pointcloud_features->size());
    }
  } else {
    spdlog::error("Unknown type: {}", type);
  }
}

pcl::PointCloud<PointT>::Ptr GlobalmapServer::origin() {
  return pointcloud_origin;
}

pcl::PointCloud<PointT>::Ptr GlobalmapServer::filtered() {
  return pointcloud_filtered;
}

pcl::PointCloud<FeatureT>::Ptr GlobalmapServer::features() {
  return pointcloud_features;
}

}  // namespace shinji