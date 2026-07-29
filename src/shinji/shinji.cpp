#include <shinji/shinji.hpp>

namespace shinji {
void Shinji::initialize(const std::string& config_file) {
  config = std::make_shared<shinji::ConfigServer>();
  config->load(config_file);

  configure_logging(config->logging);

  logger = create_module_logger("shinji", config->logging);

  globalmap = std::make_shared<shinji::GlobalmapServer>(config->common.globalmap_directory);
  globalmap->load(config->common.globalmap_origin, "origin");
  globalmap->load(config->common.globalmap_filtered, "filtered");
  globalmap->load(config->common.globalmap_features, "features");

  if (config->teaser.cost_estimation_method == "Voxels") {
    evaluater = std::make_unique<MatchingCostEvaluaterVoxels>();
  } else if (config->teaser.cost_estimation_method == "Flann") {
    evaluater = std::make_unique<MatchingCostEvaluaterFlann>();
  }

  source_frames.set_capacity(config->common.circular_buffer_capacity);
  cropbox.setNegative(config->cropbox.negative);
  cropbox.setMin(config->cropbox.min);
  cropbox.setMax(config->cropbox.max);

  setup_globalmap();
}

void Shinji::setup_globalmap() {
  const auto& origin = globalmap->origin();
  const auto& filtered = globalmap->filtered();
  const auto& features = globalmap->features();

  if (filtered->size() != features->size()) {
    throw std::runtime_error("filtered and features size mismatch");
  }

  target_cloud = std::make_unique<teaser::PointCloud>();
  target_features = std::make_unique<teaser::FPFHCloud>();
  target_cloud->reserve(filtered->size());
  target_features->reserve(features->size());

  for (int i = 0; i < filtered->size(); i++) {
    target_cloud->push_back({filtered->at(i).x, filtered->at(i).y, filtered->at(i).z});
    target_features->push_back(features->at(i));
  }

  evaluater->setup_target(filtered, config->teaser.max_correspondence_distance);

  logger->info("TEASER Initialized.");

  const auto& target_origin = origin;
  // Downsample points and convert into pcl::PointCloud<pcl::PointCovariance>.
  target_covariance = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<PointT>, pcl::PointCloud<pcl::PointCovariance>>(*target_origin, config->gicp.voxel_resolution);
  small_gicp::estimate_covariances_omp(*target_covariance, config->gicp.num_neighbors, config->gicp.num_threads);

  // Create KdTree for target.
  target_tree.reset(new small_gicp::KdTree<pcl::PointCloud<CovarianceT>>(target_covariance, small_gicp::KdTreeBuilderOMP(config->gicp.num_threads)));
  gicp_align.reduction.num_threads = config->gicp.num_threads;
  gicp_align.rejector.max_dist_sq = 1.0;

  logger->info("GICP Initialized.");
}

void Shinji::insert_frame(const Eigen::Isometry3d& pose, const pcl::PointCloud<PointT>::Ptr& cloud) {
  if (!cloud || cloud->empty()) {
    logger->warn("Attempted to insert an empty pointcloud frame. Ignoring.");
    return;
  }

  try {
    std::unique_lock<std::mutex> lock(source_frames_mutex, std::try_to_lock);
    if (!lock.owns_lock()) {
      logger->info("Failed to acquire lock on source_frames_mutex");
      return;
    }
    source_frames.push_back(AlignFrame(pose, cloud));
  } catch (const std::exception& e) {
    logger->error("Failed to insert frame: {}", e.what());
  }
}

ResultT<AlignResult> Shinji::try_request() {
  std::unique_lock<std::mutex> lock(query_mutex, std::try_to_lock);
  if (!lock.owns_lock()) {
    logger->warn("Another alignment is running, skip this request!");
    return ResultT<AlignResult>::failure(ErrorCode::TRY_LOCK_FAILED, "");
  }

  return this->query();
}

ResultT<AlignResult> Shinji::query() {
  using ClockT = std::chrono::steady_clock;

  pcl::PointCloud<PointT>::Ptr cloud4coarse_align(new pcl::PointCloud<PointT>());
  pcl::PointCloud<PointT>::Ptr cloud4fine_align(new pcl::PointCloud<PointT>());

  try {
    std::lock_guard<std::mutex> lock(source_frames_mutex);
    if (source_frames.empty()) {
      logger->warn("No source frames available!");
      return ResultT<AlignResult>::failure(ErrorCode::OTHERS, "");
    }

    if (config->teaser.enable) {
      size_t num_cloud4coarse_align{};
      for (auto it = source_frames.rbegin(); it != source_frames.rbegin() + config->teaser.source_frames; ++it) {
        num_cloud4coarse_align += (it->cloud)->size();
      }

      cloud4coarse_align->points.reserve(num_cloud4coarse_align);

      for (auto it = source_frames.rbegin(); it != source_frames.rbegin() + config->teaser.source_frames; ++it) {
        if (it->cloud && !it->cloud->empty()) {
          if (config->cropbox.enable) {
            cropbox.setInputCloud(it->cloud);
            cropbox.filter(*(it->cloud));
          }
          if (config->common.transformation_needed) {
            pcl::transformPointCloud(*(it->cloud), *(it->cloud), it->pose.matrix().cast<float>());
          }
          *cloud4coarse_align += *it->cloud;
        } else {
          logger->warn("One of the pointclouds in source_frames is empty!");
        }
      }

      if (config->common.centered) {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud4coarse_align, *cloud4coarse_align, indices);
        cloud4coarse_align->points.shrink_to_fit();
        cloud4coarse_align->is_dense = true;
      }
    }

    if (config->gicp.enable) {
      size_t num_cloud4fine_align{};
      for (auto it = source_frames.rbegin(); it != source_frames.rbegin() + config->gicp.source_frames; ++it) {
        num_cloud4fine_align += (it->cloud)->size();
      }

      cloud4fine_align->points.reserve(num_cloud4fine_align);

      for (auto it = source_frames.rbegin(); it != source_frames.rbegin() + config->gicp.source_frames; ++it) {
        if (it->cloud && !it->cloud->empty()) {
          if (config->cropbox.enable) {
            cropbox.setInputCloud(it->cloud);
            cropbox.filter(*(it->cloud));
          }
          if (config->common.transformation_needed) {
            pcl::transformPointCloud(*(it->cloud), *(it->cloud), it->pose.matrix().cast<float>());
          }
          *cloud4fine_align += *it->cloud;
        } else {
          logger->warn("One of the pointclouds in source_frames is empty!");
        }
      }

      if (config->common.centered) {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud4fine_align, *cloud4fine_align, indices);
        cloud4fine_align->points.shrink_to_fit();
        cloud4fine_align->is_dense = true;
      }
    }

    source_frames.clear();
  } catch (const std::exception& e) {
    logger->error("Failed to process input cloud: {}", e.what());
    return ResultT<AlignResult>::failure(ErrorCode::OTHERS, "");
  }

  auto cache_result = AlignResult();
  auto initial_guess = Eigen::Isometry3d::Identity();
  bool skip_coarse_align = false;

  const auto t0 = ClockT::now();
  if (config->initial_guess.enable) {
    auto guess_result = guess_verify(cloud4coarse_align);
    if (guess_result) {
      cache_result = guess_result.data;
      initial_guess = guess_result.data.pose;

      {
        const auto& result = guess_result.data;
        const auto& pose = result.pose;
        Eigen::Quaterniond quat(pose.rotation());
        logger->info("--- Guess Verification Result ---");
        logger->info("Trans : {:.3f} {:.3f} {:.3f}", pose.translation().x(), pose.translation().y(), pose.translation().z());
        logger->info("Quat  : {:.3f} {:.3f} {:.3f} {:.3f}", quat.x(), quat.y(), quat.z(), quat.w());
        logger->info("Error : {:.3f}", result.error);
        logger->info("Inlier: {:.3f}", result.inlier_fraction);
      }

      skip_coarse_align = true;

    } else {
      logger->warn("Invalid initial guess!");
      logger->warn("Message: {}", guess_result.message);
      logger->warn("Continue with teaser++ ...");
    }
  }

  if (config->teaser.enable && !skip_coarse_align) {
    auto coarse_result = coarse_align(cloud4coarse_align);
    if (coarse_result) {
      cache_result = coarse_result.data;
      initial_guess = coarse_result.data.pose;

      {
        const auto& result = coarse_result.data;
        const auto& pose = result.pose;
        Eigen::Quaterniond quat(pose.rotation());
        logger->info("--- Coarse Alignment Result ---");
        logger->info("Trans : {:.3f} {:.3f} {:.3f}", pose.translation().x(), pose.translation().y(), pose.translation().z());
        logger->info("Quat  : {:.3f} {:.3f} {:.3f} {:.3f}", quat.x(), quat.y(), quat.z(), quat.w());
        logger->info("Error : {:.3f}", result.error);
        logger->info("Inlier: {:.3f}", result.inlier_fraction);
      }

    } else {
      logger->warn("Coarse alignment failed!");
      logger->warn("Message: {}", coarse_result.message);
      return coarse_result;
    }
  }

  const auto t1 = ClockT::now();
  if (config->gicp.enable) {
    auto fine_result = fine_align(cloud4fine_align, initial_guess);
    if (fine_result) {
      cache_result = fine_result.data;

      {
        const auto& result = fine_result.data;
        const auto& pose = result.pose;
        Eigen::Quaterniond quat(pose.rotation());
        logger->info("--- Fine Alignment Result ---");
        logger->info("Trans : {:.3f} {:.3f} {:.3f}", pose.translation().x(), pose.translation().y(), pose.translation().z());
        logger->info("Quat  : {:.3f} {:.3f} {:.3f} {:.3f}", quat.x(), quat.y(), quat.z(), quat.w());
        logger->info("Error : {:.3f}", result.error);
        logger->info("Inlier: {:.3f}", result.inlier_fraction);
      }

    } else {
      logger->warn("Fine alignment failed!");
      logger->warn("Message: {}", fine_result.message);
      return fine_result;
    }
  }

  const auto t2 = ClockT::now();
  {
    const int coarse_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    const int fine_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    const int total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t0).count();
    logger->info("Timing: coarse={}, fine={}, total={}", coarse_ms, fine_ms, total_ms);
  }

  return ResultT<AlignResult>::success(std::move(cache_result));
}

ResultT<AlignResult> Shinji::guess_verify(const pcl::PointCloud<PointT>::ConstPtr& cloud) {
  if (!cloud || cloud->empty()) {
    return ResultT<AlignResult>::failure(ErrorCode::INVALID_POINTCLOUD, "empty cloud4guess_verify");
  }

  pcl::PointCloud<PointT>::Ptr filtered = voxelgrid_sampling(cloud, config->initial_guess.voxel_resolution);

  const auto& r = config->initial_guess.rotation;
  const auto& t = config->initial_guess.translation;
  auto initial_guess = Eigen::Isometry3d::Identity();
  initial_guess.linear() = Eigen::Quaterniond(r.w(), r.x(), r.y(), r.z()).toRotationMatrix();
  initial_guess.translation() = Eigen::Vector3d(t.x(), t.y(), t.z());

  Eigen::Matrix4f transformation = initial_guess.matrix().cast<float>();

  double error{};
  double inlier_fraction{};
  evaluater->estimate_matching_cost(filtered, transformation, error, inlier_fraction);

  if (inlier_fraction > config->initial_guess.inlier_fraction_threshold) {
    auto verify_result = AlignResult(error, inlier_fraction, initial_guess);
    return ResultT<AlignResult>::success(std::move(verify_result));
  } else {
    return ResultT<AlignResult>::failure(ErrorCode::INVALID_INLIER_FRACTION, "inliers->" + std::to_string(inlier_fraction));
  }
}

ResultT<AlignResult> Shinji::coarse_align(const pcl::PointCloud<PointT>::ConstPtr& cloud) {
  if (!cloud || cloud->empty()) {
    return ResultT<AlignResult>::failure(ErrorCode::INVALID_POINTCLOUD, "empty cloud4coarse_align");
  }

  Eigen::Vector4d source_centroid;
  pcl::PointCloud<PointT>::Ptr centered(new pcl::PointCloud<PointT>());
  if (config->common.centered) {
    pcl::compute3DCentroid(*cloud, source_centroid);
    pcl::demeanPointCloud(*cloud, source_centroid, *centered);
    logger->info("Source centroid: ({:.3f}, {:.3f}, {:.3f})", source_centroid.x(), source_centroid.y(), source_centroid.z());
  }

  const pcl::PointCloud<PointT>::ConstPtr& source_origin = config->common.centered ? centered : cloud;
  pcl::PointCloud<PointT>::Ptr filtered = voxelgrid_sampling(source_origin, config->teaser.voxel_resolution);
  logger->info("cloud4coarse_align size: {}", filtered->size());

  if (config->logging.enable) {
    savePCDBinary(filtered, "coarse", config->logging.pcd_saving_path);
  }

  pcl::PointCloud<FeatureT>::Ptr features = extract_features(filtered, config->fpfh.normal_estimation_radius, config->fpfh.search_radius, config->fpfh.num_threads);

  if (filtered->size() != features->size()) {
    return ResultT<AlignResult>::failure(ErrorCode::INVALID_POINTCLOUD, "filtered and features mismatch");
  }

  source_cloud = std::make_unique<teaser::PointCloud>();
  source_features = std::make_unique<teaser::FPFHCloud>();
  source_cloud->reserve(filtered->size());
  source_features->reserve(features->size());

  for (int i = 0; i < filtered->size(); i++) {
    source_cloud->push_back({filtered->at(i).x, filtered->at(i).y, filtered->at(i).z});
    source_features->push_back(features->at(i));
  }

  teaser::Matcher matcher;
  auto correspondences = matcher.calculateCorrespondences(
    *source_cloud,
    *target_cloud,
    *source_features,
    *target_features,
    false,
    config->teaser.cross_check,
    config->teaser.tuple_test,
    config->teaser.tuple_scale);

  teaser::RobustRegistrationSolver::Params params;
  params.noise_bound = config->teaser.noise_bound;
  params.cbar2 = config->teaser.cbar2;
  params.estimate_scaling = false;
  params.rotation_max_iterations = config->teaser.rotation_max_iterations;
  params.rotation_gnc_factor = config->teaser.rotation_gnc_factor;
  params.rotation_estimation_algorithm = teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
  params.rotation_cost_threshold = config->teaser.rotation_cost_threshold;

  teaser::RobustRegistrationSolver solver(params);
  solver.solve(*source_cloud, *target_cloud, correspondences);

  auto solution = solver.getSolution();

  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  pose.linear() = solution.rotation;
  pose.translation() = solution.translation;

  Eigen::Matrix4f transformation = pose.matrix().cast<float>();

  double error{};
  double inlier_fraction{};
  evaluater->estimate_matching_cost(filtered, transformation, error, inlier_fraction);

  if (inlier_fraction < config->teaser.inlier_fraction_threshold) {
    return ResultT<AlignResult>::failure(ErrorCode::INVALID_INLIER_FRACTION, "inliers->" + std::to_string(inlier_fraction));
  }

  if (config->common.centered) {
    pose.translation() += config->common.centroid.head<3>();
    pose.translation() -= source_centroid.head<3>();
  }

  auto result = AlignResult(error, inlier_fraction, pose);
  return ResultT<AlignResult>::success(std::move(result));
}

ResultT<AlignResult> Shinji::fine_align(const pcl::PointCloud<PointT>::ConstPtr& cloud, const Eigen::Isometry3d& initial_guess) {
  if (!cloud || cloud->empty()) {
    return ResultT<AlignResult>::failure(ErrorCode::INVALID_POINTCLOUD, "empty cloud4fine_align");
  }

  if (config->logging.enable) {
    pcl::PointCloud<PointT>::Ptr filtered = voxelgrid_sampling(cloud, config->gicp.voxel_resolution);
    savePCDBinary(filtered, "fine", config->logging.pcd_saving_path);
  }

  const auto& source_origin = cloud;

  // Downsample points and convert into pcl::PointCloud<pcl::PointCovariance>.
  source_covariance = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<PointT>, pcl::PointCloud<CovarianceT>>(*source_origin, config->gicp.voxel_resolution);
  small_gicp::estimate_covariances_omp(*source_covariance, config->gicp.num_neighbors, config->gicp.num_threads);
  logger->info("cloud4fine_align size: {}", source_covariance->size());

  auto solution = gicp_align.align(*target_covariance, *source_covariance, *target_tree, initial_guess);

  double error = solution.error;
  double inlier_fraction = solution.num_inliers / static_cast<double>(source_covariance->size());
  Eigen::Isometry3d pose(solution.T_target_source.matrix().cast<double>());

  if (inlier_fraction < config->gicp.inlier_fraction_threshold) {
    return ResultT<AlignResult>::failure(ErrorCode::INVALID_INLIER_FRACTION, "inliers->" + std::to_string(inlier_fraction));
  }

  auto result = AlignResult(error, inlier_fraction, pose);
  return ResultT<AlignResult>::success(std::move(result));
}

}  // namespace shinji