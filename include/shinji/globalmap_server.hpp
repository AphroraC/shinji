#pragma once
#include <string>

#include <spdlog/spdlog.h>
#include <shinji/utility.hpp>

namespace shinji {

class GlobalmapServer {
public:
  GlobalmapServer(const std::string& directory);
  GlobalmapServer() = delete;
  ~GlobalmapServer() = default;

  void load(const std::string& filename, const std::string& type);
  pcl::PointCloud<PointT>::Ptr origin();
  pcl::PointCloud<PointT>::Ptr filtered();
  pcl::PointCloud<FeatureT>::Ptr features();

private:
  std::string directory;
  pcl::PointCloud<PointT>::Ptr pointcloud_origin;
  pcl::PointCloud<PointT>::Ptr pointcloud_filtered;
  pcl::PointCloud<FeatureT>::Ptr pointcloud_features;
};

}  // namespace shinji
