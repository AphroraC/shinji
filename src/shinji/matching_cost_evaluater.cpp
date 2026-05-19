#include <shinji/matching_cost_evaluater.hpp>

namespace shinji {
void MatchingCostEvaluaterFlann::setup_target(const pcl::PointCloud<PointT>::ConstPtr& cloud, double max_correspondence_distance) {
  max_correspondence_distance_sq = max_correspondence_distance * max_correspondence_distance;

  tree.reset(new pcl::KdTreeFLANN<PointT>);
  tree->setInputCloud(cloud);
}

void MatchingCostEvaluaterFlann::estimate_matching_cost(
  const pcl::PointCloud<PointT>::ConstPtr& cloud,
  const Eigen::Matrix4f& transformation,
  double& error,
  double& inlier_fraction) {
  int num_inliers{};
  double matching_error{};

  pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());
  pcl::transformPointCloud(*cloud, *transformed, transformation);

  std::vector<int> indices;
  std::vector<float> sq_dists;
  for (int i = 0; i < transformed->size(); i++) {
    tree->nearestKSearch((*transformed)[i], 1, indices, sq_dists);
    if (sq_dists[0] < max_correspondence_distance_sq) {
      num_inliers++;
      matching_error += sq_dists[0];
    }
  }

  error = num_inliers ? matching_error / num_inliers : std::numeric_limits<double>::max();
  inlier_fraction = static_cast<double>(num_inliers) / cloud->size();
  return;
}

void MatchingCostEvaluaterVoxels::setup_target(const pcl::PointCloud<PointT>::ConstPtr& cloud, double max_correspondence_distance) {
  voxels = std::make_unique<VoxelSet>(max_correspondence_distance);
  voxels->setup_target(cloud);
}

void MatchingCostEvaluaterVoxels::estimate_matching_cost(
  const pcl::PointCloud<PointT>::ConstPtr& cloud,
  const Eigen::Matrix4f& transformation,
  double& error,
  double& inlier_fraction) {
  pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());
  pcl::transformPointCloud(*cloud, *transformed, transformation);

  voxels->estimate_matching_cost(transformed, error, inlier_fraction);
  return;
}

}  // namespace shinji