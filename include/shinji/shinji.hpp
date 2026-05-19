#pragma once
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/circular_buffer.hpp>

#include <spdlog/spdlog.h>

#include <teaser/matcher.h>
#include <teaser/registration.h>

#include <shinji/config_server.hpp>
#include <shinji/globalmap_server.hpp>
#include <shinji/matching_cost_evaluater.hpp>
#include <shinji/logging.hpp>
#include <shinji/utility.hpp>

namespace shinji {
class Shinji {
public:
  Shinji() = default;
  ~Shinji() = default;

  void initialize(const std::string& config_file);
  void insert_frame(const Eigen::Isometry3d& pose, const pcl::PointCloud<PointT>::Ptr& cloud);
  ResultT<AlignResult> try_request();
  std::shared_ptr<const shinji::ConfigServer> config_server() const { return config; }

private:
  void setup_globalmap();
  ResultT<AlignResult> coarse_align(const pcl::PointCloud<PointT>::ConstPtr& cloud);
  ResultT<AlignResult> fine_align(const pcl::PointCloud<PointT>::ConstPtr& cloud, const Eigen::Isometry3d& initial_guess);
  ResultT<AlignResult> query();

private:
  std::unique_ptr<teaser::PointCloud> target_cloud;
  std::unique_ptr<teaser::PointCloud> source_cloud;
  std::unique_ptr<teaser::FPFHCloud> target_features;
  std::unique_ptr<teaser::FPFHCloud> source_features;
  std::unique_ptr<shinji::MatchingCostEvaluater> evaluater;

  pcl::PointCloud<CovarianceT>::Ptr target_covariance;
  pcl::PointCloud<CovarianceT>::Ptr source_covariance;
  small_gicp::KdTree<pcl::PointCloud<CovarianceT>>::Ptr target_tree;
  small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> gicp_align;

  std::mutex source_frames_mutex;
  boost::circular_buffer<shinji::AlignFrame> source_frames;
  pcl::CropBox<PointT> cropbox;

  std::shared_ptr<shinji::ConfigServer> config;
  std::shared_ptr<shinji::GlobalmapServer> globalmap;

  std::mutex query_mutex;
  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace shinji
