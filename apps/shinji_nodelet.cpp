#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include <shinji/shinji.hpp>

namespace shinji {
class ShinjiNodelet : public nodelet::Nodelet {
public:
  ShinjiNodelet() = default;
  virtual ~ShinjiNodelet() override;
  void onInit() override;

private:
  void setupConfigurations();
  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void publishTransform(const ros::Time& stamp, const Eigen::Isometry3d& pose);
  bool serviceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& respond);

private:
  ros::NodeHandle pnh;
  ros::ServiceServer server;

  std::string cloud_topic;
  std::string globalmap_frame;
  std::string lidar_odom_frame;
  std::string lidar_base_frame;
  bool publish_map2odom_tf;
  bool transformation_needed;
  double tf_listen_timeout;

  geometry_msgs::TransformStamped tf_msg;
  tf2_ros::Buffer tf_buffer{};
  tf2_ros::TransformListener tf_listener{tf_buffer};
  tf2_ros::TransformBroadcaster tf_broadcaster;

  ros::Subscriber pointcloud_subscriber;
  ros::Publisher transform_publisher;

  std::atomic_bool tf_thread_running{false};
  std::thread tf_thread;
  std::mutex tf_mutex;

  std::unique_ptr<shinji::Shinji> shinji;
};

ShinjiNodelet::~ShinjiNodelet() {
  tf_thread_running.store(false);
  if (tf_thread.joinable()) {
    tf_thread.join();
  }
}

void ShinjiNodelet::setupConfigurations() {
  const auto& config = shinji->config_server();

  cloud_topic = config->ros.cloud_topic;
  globalmap_frame = config->ros.globalmap_frame;
  lidar_odom_frame = config->ros.lidar_odom_frame;
  lidar_base_frame = config->ros.lidar_base_frame;
  publish_map2odom_tf = config->ros.publish_map2odom_tf;
  transformation_needed = config->common.transformation_needed;
  tf_listen_timeout = config->ros.tf_listen_timeout;

  tf_msg.header.frame_id = globalmap_frame;
  tf_msg.child_frame_id = lidar_odom_frame;
  tf_msg.transform.rotation.w = 1.0;
}

void ShinjiNodelet::onInit() {
  pnh = getPrivateNodeHandle();

  const std::string config_file = pnh.param<std::string>("config_file", "");
  shinji = std::make_unique<shinji::Shinji>();
  shinji->initialize(config_file);

  setupConfigurations();

  pointcloud_subscriber = pnh.subscribe(cloud_topic, 5, &ShinjiNodelet::pointcloudCallback, this);
  transform_publisher = pnh.advertise<geometry_msgs::TransformStamped>("/shinji/result", 5, false);
  server = pnh.advertiseService("/shinji/query", &ShinjiNodelet::serviceCallback, this);

  if (publish_map2odom_tf) {
    tf_thread_running.store(true);
    tf_thread = std::thread([this]() {
      ros::Rate rate(10.0);  // 10 Hz
      while (tf_thread_running.load() && ros::ok()) {
        {
          std::lock_guard<std::mutex> lock(tf_mutex);
          tf_msg.header.stamp = ros::Time::now();
          tf_broadcaster.sendTransform(tf_msg);
        }
        rate.sleep();
      }
    });
  }
}

void ShinjiNodelet::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};

  if (transformation_needed) {
    try {
      const auto& tf = tf_buffer.lookupTransform(lidar_odom_frame, lidar_base_frame, msg->header.stamp, ros::Duration(tf_listen_timeout));
      pose.translation() << tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z;
      pose.rotate(Eigen::Quaterniond(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z));
    } catch (const tf2::TransformException& ex) {
      NODELET_WARN("Transform lookup failed: %s", ex.what());
      return;
    }
  }

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg(*msg, *cloud);

  shinji->insert_frame(pose, cloud);

  return;
}

void ShinjiNodelet::publishTransform(const ros::Time& stamp, const Eigen::Isometry3d& pose) {
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = globalmap_frame;
  msg.child_frame_id = lidar_odom_frame;

  msg.transform.translation.x = pose.translation().x();
  msg.transform.translation.y = pose.translation().y();
  msg.transform.translation.z = pose.translation().z();

  Eigen::Quaterniond quat(pose.rotation());
  msg.transform.rotation.x = quat.x();
  msg.transform.rotation.y = quat.y();
  msg.transform.rotation.z = quat.z();
  msg.transform.rotation.w = quat.w();

  transform_publisher.publish(msg);

  if (publish_map2odom_tf) {
    std::unique_lock<std::mutex> lock(tf_mutex, std::try_to_lock);
    tf_msg = msg;
  }
}

bool ShinjiNodelet::serviceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& respond) {
  std::thread([this]() {
    const auto result = shinji->try_request();

    if (result) {
      auto time = ros::Time::now();
      publishTransform(time, result.data.pose);
    }
  }).detach();
  return true;
}

}  // namespace shinji

PLUGINLIB_EXPORT_CLASS(shinji::ShinjiNodelet, nodelet::Nodelet)