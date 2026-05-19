#pragma once

#include <functional>
#ifdef USE_UNORDERED_DENSE
#include <ankerl/unordered_dense.h>
#else
#include <unordered_set>
#endif

#include <shinji/utility.hpp>

namespace shinji {
class Vector3iHash {
public:
  size_t operator()(const Eigen::Vector3i& x) const {
    const size_t hash1 = std::hash<int>()(x[0]);
    const size_t hash2 = std::hash<int>()(x[1]);
    const size_t hash3 = std::hash<int>()(x[2]);

    return hash1 ^ (hash2 << 1) ^ (hash3 << 2);
  }
};

class VoxelSet {
public:
  VoxelSet(double resolution) : resolution(resolution) {}

  void setup_target(const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    voxels.clear();
    voxels.reserve(cloud->size());

    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> offsets;
    offsets.reserve(27);
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        for (int k = -1; k <= 1; k++) {
          offsets.push_back(Eigen::Vector3i(i, j, k));
        }
      }
    }

    for (const auto& pt : *cloud) {
      Eigen::Vector3i coord = voxel_coord(pt.getVector4fMap());
      for (const auto& offset : offsets) {
        Eigen::Vector4f center = voxel_center(coord + offset);
        if ((center - pt.getVector4fMap()).squaredNorm() < resolution * resolution) {
          voxels.insert(coord + offset);
        }
      }
    }

    return;
  }

  void estimate_matching_cost(const pcl::PointCloud<PointT>::ConstPtr& cloud, double& error, double& inlier_fraction) const {
    int num_inliers{};
    double matching_error{};

    for (const auto& pt : *cloud) {
      Eigen::Vector3i coord = voxel_coord(pt.getVector4fMap());
      if (voxels.find(coord) != voxels.end()) {
        Eigen::Vector4f center = voxel_center(coord);
        num_inliers++;
        matching_error += (pt.getVector4fMap() - center).squaredNorm();
      }
    }

    error = num_inliers ? matching_error / num_inliers : std::numeric_limits<double>::max();
    inlier_fraction = static_cast<double>(num_inliers) / cloud->size();

    return;
  }

private:
  Eigen::Vector3i voxel_coord(const Eigen::Vector4f& x) const { return (x.array() / resolution - 0.5).floor().cast<int>().head<3>(); }
  Eigen::Vector4f voxel_center(const Eigen::Vector3i& coord) const {
    float x = (coord[0] + 0.5f) * resolution;
    float y = (coord[1] + 0.5f) * resolution;
    float z = (coord[2] + 0.5f) * resolution;
    return Eigen::Vector4f(x, y, z, 1.0f);
  }

private:
  double resolution;

#ifdef USE_UNORDERED_DENSE
  using VoxelType = ankerl::unordered_dense::set<Eigen::Vector3i, Vector3iHash, std::equal_to<Eigen::Vector3i>, Eigen::aligned_allocator<Eigen::Vector3i>>;

#else
  using VoxelType = std::unordered_set<Eigen::Vector3i, Vector3iHash, std::equal_to<Eigen::Vector3i>, Eigen::aligned_allocator<Eigen::Vector3i>>;
#endif

  VoxelType voxels;
};

}  // namespace shinji