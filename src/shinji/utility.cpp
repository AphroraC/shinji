#include <shinji/utility.hpp>

namespace shinji {
pcl::PointCloud<PointT>::Ptr voxelgrid_sampling(const pcl::PointCloud<PointT>::ConstPtr& cloud, double resolution) {
  if (!cloud || cloud->empty()) {
    return pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
  }

  pcl::PointCloud<PointT>::Ptr cleaned(new pcl::PointCloud<PointT>);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cleaned, indices);
  if (cleaned->empty() || resolution <= 0.0) {
    return cleaned;
  }

  pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
  pcl::ApproximateVoxelGrid<PointT> voxelgrid;
  voxelgrid.setLeafSize(resolution, resolution, resolution);
  voxelgrid.setInputCloud(cleaned);
  voxelgrid.filter(*filtered);
  filtered->header = cloud->header;
  filtered->is_dense = true;

  return filtered;
}

pcl::PointCloud<FeatureT>::Ptr
extract_features(const pcl::PointCloud<PointT>::ConstPtr& cloud, double normal_estimation_radius = 1.0, double search_radius = 2.0, int num_threads = 10) {
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimationOMP<PointT, pcl::Normal> nest;
  nest.setNumberOfThreads(num_threads);
  nest.setRadiusSearch(normal_estimation_radius);
  nest.setInputCloud(cloud);
  nest.compute(*normals);

  pcl::PointCloud<FeatureT>::Ptr features(new pcl::PointCloud<FeatureT>);
  pcl::FPFHEstimationOMP<PointT, pcl::Normal, FeatureT> fest;
  fest.setNumberOfThreads(num_threads);
  fest.setRadiusSearch(search_radius);
  fest.setInputCloud(cloud);
  fest.setInputNormals(normals);
  fest.compute(*features);

  return features;
}

pcl::PointCloud<PointT>::Ptr vector2pointcloud(const std::vector<Eigen::Vector4d>& vector) {
  static_assert(std::is_same_v<PointT, pcl::PointXYZI>, "vector2pointcloud assumes PointXYZI (x,y,z,intensity) layout");

  if (vector.empty()) {
    return pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
  }

  const size_t n = vector.size();
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  cloud->points.reserve(n);

  for (const auto& p : vector) {
    auto& pt = cloud->points.emplace_back();
    pt.x = static_cast<float>(p.x());
    pt.y = static_cast<float>(p.y());
    pt.z = static_cast<float>(p.z());
    pt.intensity = static_cast<float>(p.w());
  }

  cloud->width = static_cast<uint32_t>(n);
  cloud->height = 1;
  cloud->is_dense = true;

  return cloud;
}

std::vector<Eigen::Vector4d> pointcloud2vector(const pcl::PointCloud<PointT>::ConstPtr& cloud) {
  if (!cloud || cloud->empty()) {
    return {};
  }

  const size_t n = cloud->size();
  std::vector<Eigen::Vector4d> vector;
  vector.reserve(n);

  for (size_t i = 0; i < n; ++i) {
    const auto& p = cloud->points[i];
    vector.emplace_back(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z), 1.0);
  }
  return vector;
}

void savePCDBinary(const pcl::PointCloud<PointT>::Ptr& cloud, const std::string& cloud_type, const std::string& save_path) {
  // Check if cloud is valid
  if (!cloud || cloud->empty()) {
    std::cerr << "Error: Invalid or empty point cloud" << std::endl;
    return;
  }

  const std::string& filepath = save_path;

  // Check if directory exists, create if not
  std::filesystem::path dir_path(filepath);
  if (!std::filesystem::exists(dir_path)) {
    try {
      std::filesystem::create_directories(dir_path);
      std::cout << "Created directory: " << filepath << std::endl;
    } catch (const std::filesystem::filesystem_error& e) {
      std::cerr << "Failed to create directory " << filepath << ": " << e.what() << std::endl;
      return;
    }
  }

  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");

  // Sanitize cloud_type to prevent path injection
  std::string safe_cloud_type = cloud_type;
  safe_cloud_type.erase(std::remove_if(safe_cloud_type.begin(), safe_cloud_type.end(), [](char c) { return c == '/' || c == '\\' || c == '.'; }), safe_cloud_type.end());

  std::string filename = filepath;
  if (!filename.empty() && filename.back() != '/') {
    filename += "/";
  }
  filename += safe_cloud_type + "_" + ss.str() + ".pcd";

  cloud->width = static_cast<uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = true;

  try {
    if (pcl::io::savePCDFileBinary(filename, *cloud) == 0) {
      // std::cout << "Saved pointcloud to: " << filename << std::endl;
    } else {
      std::cerr << "Failed to save pointcloud to: " << filename << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "Exception while saving pointcloud: " << e.what() << std::endl;
  }
}
}  // namespace shinji