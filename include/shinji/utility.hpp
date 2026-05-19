#pragma once
#include <cstdint>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/factors/plane_icp_factor.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>

namespace shinji {
using namespace small_gicp;
using PointT = pcl::PointXYZI;
using FeatureT = pcl::FPFHSignature33;
using CovarianceT = pcl::PointCovariance;

pcl::PointCloud<PointT>::Ptr voxelgrid_sampling(const pcl::PointCloud<PointT>::ConstPtr& cloud, double resolution);
pcl::PointCloud<FeatureT>::Ptr extract_features(const pcl::PointCloud<PointT>::ConstPtr& cloud, double normal_estimation_radius, double search_radius, int num_threads);
pcl::PointCloud<PointT>::Ptr vector2pointcloud(const std::vector<Eigen::Vector4d>& vector);
std::vector<Eigen::Vector4d> pointcloud2vector(const pcl::PointCloud<PointT>::ConstPtr& cloud);
void savePCDBinary(const pcl::PointCloud<PointT>::Ptr& cloud, const std::string& cloud_type, const std::string& save_path);

struct AlignFrame {
  AlignFrame() : pose(Eigen::Isometry3d::Identity()), cloud(nullptr) {}
  AlignFrame(const Eigen::Isometry3d& pose, const pcl::PointCloud<PointT>::Ptr& cloud) : pose(pose), cloud(cloud) {}
  Eigen::Isometry3d pose;
  pcl::PointCloud<PointT>::Ptr cloud;
};

struct AlignResult {
  AlignResult() : error(0.), inlier_fraction(0.), pose(Eigen::Isometry3d::Identity()) {}
  AlignResult(double error, double inlier_fraction, const Eigen::Isometry3d& pose) : error(error), inlier_fraction(inlier_fraction), pose(pose) {}
  double error;
  double inlier_fraction;
  Eigen::Isometry3d pose;
};

enum class ErrorCode : std::uint8_t { NONE = 0, TRY_LOCK_FAILED, INVALID_POINTCLOUD, INVALID_INLIER_FRACTION, OTHERS };

template <typename DataT>
struct ResultT {
  bool ok;
  DataT data;
  ErrorCode error;
  std::string message;

  static ResultT<DataT> success(const DataT& data) { return {true, data, ErrorCode::NONE, ""}; }
  static ResultT<DataT> success(DataT&& data) { return {true, std::move(data), ErrorCode::NONE, ""}; }
  static ResultT<DataT> failure(ErrorCode error, const std::string& msg) { return {false, {}, error, msg}; }

  explicit operator bool() const { return ok; }
};

}  // namespace shinji