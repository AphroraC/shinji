#pragma once

#include <shinji/globalmap_server.hpp>
#include <shinji/voxelset.hpp>
#include <shinji/utility.hpp>

namespace shinji {
class MatchingCostEvaluater {
public:
  MatchingCostEvaluater() {}
  virtual ~MatchingCostEvaluater() {}

  virtual void setup_target(const pcl::PointCloud<PointT>::ConstPtr& cloud, double max_correspondence_distance) = 0;
  virtual void estimate_matching_cost(const pcl::PointCloud<PointT>::ConstPtr& cloud, const Eigen::Matrix4f& transformation, double& error, double& inlier_fraction) = 0;
};

class MatchingCostEvaluaterFlann : public MatchingCostEvaluater {
public:
  MatchingCostEvaluaterFlann() {}
  virtual ~MatchingCostEvaluaterFlann() override {}

  virtual void setup_target(const pcl::PointCloud<PointT>::ConstPtr& cloud, double max_correspondence_distance) override;
  virtual void estimate_matching_cost(const pcl::PointCloud<PointT>::ConstPtr& cloud, const Eigen::Matrix4f& transformation, double& error, double& inlier_fraction) override;

private:
  double max_correspondence_distance_sq;
  pcl::KdTreeFLANN<PointT>::Ptr tree;
};

class MatchingCostEvaluaterVoxels : public MatchingCostEvaluater {
public:
  MatchingCostEvaluaterVoxels() {}
  virtual ~MatchingCostEvaluaterVoxels() override {}

  virtual void setup_target(const pcl::PointCloud<PointT>::ConstPtr& cloud, double max_correspondence_distance) override;
  virtual void estimate_matching_cost(const pcl::PointCloud<PointT>::ConstPtr& cloud, const Eigen::Matrix4f& transformation, double& error, double& inlier_fraction) override;

private:
  std::unique_ptr<VoxelSet> voxels;
};

}  // namespace shinji