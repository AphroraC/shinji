#include <iostream>
#include <string>
#include <chrono>
#include <iomanip>
#include <thread>
#include <filesystem>
#include <yaml-cpp/yaml.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

using PointT = pcl::PointXYZI;
using FeatureT = pcl::FPFHSignature33;

class Timer {
  using Clock = std::chrono::steady_clock;

private:
  std::string name;
  Clock::time_point start;

public:
  Timer(const std::string& name) : name(name) {
    start = Clock::now();
    std::cout << "[" << name << "] Starting..." << std::endl;
  }

  ~Timer() {
    auto end = Clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "[" << name << "] Completed in " << std::fixed << std::setprecision(2) << duration.count() << " ms (" << duration.count() / 1000.0 << " seconds)" << std::endl;
  }
};

void validateConfig(const YAML::Node& config) {
  if (!config["globalmap_path"]) {
    throw std::runtime_error("Missing 'globalmap_path' in config");
  }
  if (!config["voxel_resolution"]) {
    throw std::runtime_error("Missing 'voxel_resolution' in config");
  }

  double voxel_res = config["voxel_resolution"].as<double>();
  if (voxel_res <= 0.0 || voxel_res > 2.0) {
    throw std::runtime_error("Invalid voxel_resolution: " + std::to_string(voxel_res));
  }

  double normal_radius = config["fpfh_normal_estimation_radius"].as<double>(0.5);
  double fpfh_radius = config["fpfh_search_radius"].as<double>(2.0);
  if (fpfh_radius <= normal_radius) {
    std::cout << "Warning: FPFH search radius should be larger than normal estimation radius" << std::endl;
  }
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <config_file.yaml>" << std::endl;
    return -1;
  }

  try {
    std::cout << "=== Global Map Point Cloud Preprocessing ===" << std::endl;

    // Load and validate configuration
    {
      Timer timer("Config Loading");
      YAML::Node config = YAML::LoadFile(argv[1]);
      validateConfig(config);

      std::string globalmap_path = config["globalmap_path"].as<std::string>();
      double voxel_resolution = config["voxel_resolution"].as<double>();
      double fpfh_normal_estimation_radius = config["fpfh_normal_estimation_radius"].as<double>(0.5);
      double fpfh_search_radius = config["fpfh_search_radius"].as<double>(2.0);
      int fpfh_num_threads = config["fpfh_num_threads"].as<int>(std::thread::hardware_concurrency());
      bool enable_outlier_removal = config["enable_outlier_removal"].as<bool>(false);
      bool centroid_subtraction = config["centroid_subtraction"].as<bool>(false);
      std::string output_dir = config["output_directory"].as<std::string>("./");

      std::cout << "Configuration:" << std::endl;
      std::cout << "  Input file: " << globalmap_path << std::endl;
      std::cout << "  Voxel resolution: " << voxel_resolution << " m" << std::endl;
      std::cout << "  Normal radius: " << fpfh_normal_estimation_radius << " m" << std::endl;
      std::cout << "  FPFH radius: " << fpfh_search_radius << " m" << std::endl;
      std::cout << "  Threads: " << fpfh_num_threads << std::endl;
      std::cout << "  Outlier removal: " << (enable_outlier_removal ? "enabled" : "disabled") << std::endl;
      std::cout << "  Centroid subtraction: " << (centroid_subtraction ? "enabled" : "disabled") << std::endl;
      std::cout << "  Output directory: " << output_dir << std::endl;

      // Create output directory if it doesn't exist
      std::filesystem::create_directories(output_dir);

      // Load point cloud
      pcl::PointCloud<PointT>::Ptr globalmap_origin(new pcl::PointCloud<PointT>());
      {
        Timer timer("Point Cloud Loading");
        if (pcl::io::loadPCDFile<PointT>(globalmap_path, *globalmap_origin) == -1) {
          throw std::runtime_error("Couldn't read file: " + globalmap_path);
        }

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*globalmap_origin, *globalmap_origin, indices);
        globalmap_origin->points.shrink_to_fit();
        globalmap_origin->is_dense = true;

        // Check if point cloud is valid
        if (globalmap_origin->empty()) {
          throw std::runtime_error("Loaded point cloud is empty");
        }

        if (centroid_subtraction) {
          Eigen::Vector4d centroid;
          pcl::compute3DCentroid(*globalmap_origin, centroid);
          pcl::demeanPointCloud(*globalmap_origin, centroid, *globalmap_origin);

          std::cout << "Subtracted centroid: " << centroid.transpose() << std::endl;

          std::string filtered_path = output_dir + "centered_origin.pcd";
          pcl::io::savePCDFileBinary(filtered_path, *globalmap_origin);
        }

        std::cout << "Loaded " << globalmap_origin->size() << " points" << std::endl;

        // Print point cloud bounds
        PointT min_pt, max_pt;
        pcl::getMinMax3D(*globalmap_origin, min_pt, max_pt);
        std::cout << "Point cloud bounds: [" << min_pt.x << ", " << min_pt.y << ", " << min_pt.z << "] to [" << max_pt.x << ", " << max_pt.y << ", " << max_pt.z << "]"
                  << std::endl;
      }

      // Optional: Remove outliers
      if (enable_outlier_removal) {
        Timer timer("Statistical Outlier Removal");
        pcl::PointCloud<PointT>::Ptr filtered_outliers(new pcl::PointCloud<PointT>);
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(globalmap_origin);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*filtered_outliers);

        size_t removed = globalmap_origin->size() - filtered_outliers->size();
        std::cout << "Removed " << removed << " outliers (" << std::fixed << std::setprecision(1) << (double)removed / globalmap_origin->size() * 100.0 << "%)" << std::endl;
        globalmap_origin = filtered_outliers;
      }

      // Downsample point cloud
      pcl::PointCloud<PointT>::Ptr globalmap_filtered(new pcl::PointCloud<PointT>());
      {
        Timer timer("Voxel Grid Downsampling");
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(globalmap_origin);
        voxel_filter.setLeafSize(voxel_resolution, voxel_resolution, voxel_resolution);
        voxel_filter.filter(*globalmap_filtered);

        double reduction_ratio = (1.0 - (double)globalmap_filtered->size() / (double)globalmap_origin->size()) * 100.0;
        std::cout << "Downsampled to " << globalmap_filtered->size() << " points" << std::endl;
        std::cout << "Reduction ratio: " << std::fixed << std::setprecision(1) << reduction_ratio << "%" << std::endl;

        if (centroid_subtraction) {
          std::string filtered_path = output_dir + "centered_filtered.pcd";
          pcl::io::savePCDFileBinary(filtered_path, *globalmap_filtered);
        } else {
          std::string filtered_path = output_dir + "globalmap_filtered.pcd";
          pcl::io::savePCDFileBinary(filtered_path, *globalmap_filtered);
        }
      }

      // Free original point cloud memory early
      globalmap_origin.reset();

      // Estimate normals for FPFH
      pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
      {
        Timer timer("Normal Estimation for FPFH");
        pcl::NormalEstimationOMP<PointT, pcl::Normal> normal_estimation;
        normal_estimation.setNumberOfThreads(fpfh_num_threads);
        normal_estimation.setInputCloud(globalmap_filtered);

        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        normal_estimation.setSearchMethod(tree);
        normal_estimation.setRadiusSearch(fpfh_normal_estimation_radius);
        normal_estimation.compute(*normals);

        std::cout << "Estimated normals for " << normals->size() << " points" << std::endl;

        // Check for invalid normals
        size_t invalid_normals = 0;
        for (const auto& normal : normals->points) {
          if (!std::isfinite(normal.normal_x) || !std::isfinite(normal.normal_y) || !std::isfinite(normal.normal_z)) {
            invalid_normals++;
          }
        }
        if (invalid_normals > 0) {
          std::cout << "Warning: " << invalid_normals << " invalid normals detected (" << std::fixed << std::setprecision(1) << (invalid_normals * 100.0 / normals->size()) << "%)"
                    << std::endl;
        }
      }

      // Compute FPFH features
      pcl::PointCloud<FeatureT>::Ptr fpfh_features(new pcl::PointCloud<FeatureT>);
      {
        Timer timer("FPFH Feature Extraction");
        pcl::FPFHEstimationOMP<PointT, pcl::Normal, FeatureT> fpfh_estimation;
        fpfh_estimation.setNumberOfThreads(fpfh_num_threads);
        fpfh_estimation.setInputCloud(globalmap_filtered);
        fpfh_estimation.setInputNormals(normals);

        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        fpfh_estimation.setSearchMethod(tree);
        fpfh_estimation.setRadiusSearch(fpfh_search_radius);
        fpfh_estimation.compute(*fpfh_features);

        std::cout << "Computed FPFH features for " << fpfh_features->size() << " points" << std::endl;

        // Verify feature sizes match
        if (globalmap_filtered->size() == fpfh_features->size()) {
          std::cout << "✓ Point cloud and FPFH feature sizes match!" << std::endl;
        } else {
          std::cout << "⚠ Warning: Point cloud size (" << globalmap_filtered->size() << ") != FPFH features size (" << fpfh_features->size() << ")" << std::endl;
        }

        if (centroid_subtraction) {
          std::string features_path = output_dir + "centered_features.pcd";
          pcl::io::savePCDFileBinary(features_path, *fpfh_features);
        } else {
          std::string features_path = output_dir + "globalmap_features.pcd";
          pcl::io::savePCDFileBinary(features_path, *fpfh_features);
        }
      }

      std::cout << "\n=== Processing Summary ===" << std::endl;
      std::cout << "Generated files:" << std::endl;
      if (centroid_subtraction) {
        std::cout << "  1. " << output_dir << "centered_filtered.pcd (PCL standard downsampled)" << std::endl;
        std::cout << "  2. " << output_dir << "centered_features.pcd (FPFH features)" << std::endl;
      } else {
        std::cout << "  1. " << output_dir << "globalmap_filtered.pcd (PCL standard downsampled)" << std::endl;
        std::cout << "  2. " << output_dir << "globalmap_features.pcd (FPFH features)" << std::endl;
      }

      std::cout << "\nPoint counts:" << std::endl;
      std::cout << "  Standard filtered: " << globalmap_filtered->size() << " points" << std::endl;
      std::cout << "  FPFH features:     " << fpfh_features->size() << " features" << std::endl;

      std::cout << "\n=== Processing Complete ===" << std::endl;
    }

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}