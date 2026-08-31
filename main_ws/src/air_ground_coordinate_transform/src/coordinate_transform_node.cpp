#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace air_ground_coordinate_transform {

class CoordinateTransformNode {
 public:
  CoordinateTransformNode() : nh_(), private_nh_("~") {
    LoadParameters();

    uav_odom_sub_ = nh_.subscribe(uav_odom_topic_, 10, &CoordinateTransformNode::UavOdomCallback, this);
    ugv_odom_sub_ = nh_.subscribe(ugv_odom_topic_, 10, &CoordinateTransformNode::UgvOdomCallback, this);
    observation_sub_ = nh_.subscribe(observation_topic_, 20, &CoordinateTransformNode::ObservationCallback, this);

    transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("/coordinate_transform/uav_to_ugv", 1, true);
    fused_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "/coordinate_transform/ugv_pose_in_uav_odom", 1, true);
    valid_pub_ = nh_.advertise<std_msgs::Bool>("/coordinate_transform/valid", 1, true);
    count_pub_ = nh_.advertise<std_msgs::UInt32>("/coordinate_transform/observation_count", 1, true);

    PublishStaticExtrinsics();
    publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_hz_),
                                     &CoordinateTransformNode::PublishTimerCallback, this);
  }

 private:
  void LoadParameters() {
    private_nh_.param<std::string>("uav_odom_topic", uav_odom_topic_, "/iris_0/mavros/local_position/odom");
    private_nh_.param<std::string>("ugv_odom_topic", ugv_odom_topic_, "/ugv_0/odom");
    private_nh_.param<std::string>("observation_topic", observation_topic_, "/iris_0/ugv_observation");
    private_nh_.param<std::string>("uav_odom_frame", uav_odom_frame_, "iris_0/odom");
    private_nh_.param<std::string>("uav_base_frame", uav_base_frame_, "iris_0/base_link");
    private_nh_.param<std::string>("camera_frame", camera_frame_, "iris_0/camera_link");
    private_nh_.param<std::string>("ugv_odom_frame", ugv_odom_frame_, "ugv_0/odom");
    private_nh_.param<std::string>("ugv_base_frame", ugv_base_frame_, "ugv_0/base_link");
    private_nh_.param<std::string>("ugv_marker_frame", ugv_marker_frame_, "ugv_0/fiducial");
    private_nh_.param<std::string>("ugv_camera_frame", ugv_camera_frame_, "ugv_0/camera_link");
    private_nh_.param("sync_slop_sec", sync_slop_sec_, 0.10);
    private_nh_.param("minimum_observations", minimum_observations_, 20);
    private_nh_.param("max_observation_distance_m", max_observation_distance_m_, 8.0);
    private_nh_.param("max_sample_translation_error_m", max_sample_translation_error_m_, 0.50);
    private_nh_.param("max_sample_rotation_error_rad", max_sample_rotation_error_rad_, 0.35);
    private_nh_.param("publish_rate_hz", publish_rate_hz_, 20.0);

    uav_base_to_camera_ = LoadRigidTransform("uav_base_to_camera");
    ugv_base_to_marker_ = LoadRigidTransform("ugv_base_to_marker");
    ugv_base_to_camera_ = LoadRigidTransform("ugv_base_to_camera");
  }

  tf2::Transform LoadRigidTransform(const std::string& prefix) {
    std::vector<double> translation;
    std::vector<double> rpy;
    if (!private_nh_.getParam(prefix + "_translation", translation) || translation.size() != 3 ||
        !private_nh_.getParam(prefix + "_rpy", rpy) || rpy.size() != 3) {
      throw std::runtime_error("Expected three-element " + prefix + " translation and rpy parameters");
    }
    tf2::Quaternion rotation;
    rotation.setRPY(rpy[0], rpy[1], rpy[2]);
    return tf2::Transform(rotation, tf2::Vector3(translation[0], translation[1], translation[2]));
  }

  static tf2::Transform PoseToTransform(const geometry_msgs::Pose& pose) {
    tf2::Transform transform;
    tf2::fromMsg(pose, transform);
    return transform;
  }

  geometry_msgs::TransformStamped MakeTransformStamped(const tf2::Transform& transform,
                                                       const std::string& parent,
                                                       const std::string& child,
                                                       const ros::Time& stamp) const {
    geometry_msgs::TransformStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = parent;
    message.child_frame_id = child;
    message.transform = tf2::toMsg(transform);
    return message;
  }

  void PublishStaticExtrinsics() {
    const ros::Time stamp = ros::Time::now();
    static_broadcaster_.sendTransform({
        MakeTransformStamped(uav_base_to_camera_, uav_base_frame_, camera_frame_, stamp),
        MakeTransformStamped(ugv_base_to_marker_, ugv_base_frame_, ugv_marker_frame_, stamp),
        MakeTransformStamped(ugv_base_to_camera_, ugv_base_frame_, ugv_camera_frame_, stamp)});
  }

  void UavOdomCallback(const nav_msgs::OdometryConstPtr& message) {
    const tf2::Transform pose = PoseToTransform(message->pose.pose);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      uav_odom_ = *message;
      has_uav_odom_ = true;
    }
    dynamic_broadcaster_.sendTransform(MakeTransformStamped(
        pose, uav_odom_frame_, uav_base_frame_, ValidStamp(message->header.stamp)));
  }

  void UgvOdomCallback(const nav_msgs::OdometryConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    ugv_odom_ = *message;
    has_ugv_odom_ = true;
  }

  void ObservationCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& observation) {
    nav_msgs::Odometry uav_odom;
    nav_msgs::Odometry ugv_odom;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_uav_odom_ || !has_ugv_odom_) {
        return;
      }
      uav_odom = uav_odom_;
      ugv_odom = ugv_odom_;
    }

    const ros::Time stamp = ValidStamp(observation->header.stamp);
    if (std::abs((uav_odom.header.stamp - stamp).toSec()) > sync_slop_sec_ ||
        std::abs((ugv_odom.header.stamp - stamp).toSec()) > sync_slop_sec_) {
      ROS_WARN_THROTTLE(2.0, "Ignoring unsynchronised UAV/UGV odometry for visual observation");
      return;
    }
    if (!observation->header.frame_id.empty() && observation->header.frame_id != camera_frame_) {
      ROS_WARN_THROTTLE(2.0, "Ignoring observation in frame '%s'; expected '%s'",
                        observation->header.frame_id.c_str(), camera_frame_.c_str());
      return;
    }

    const tf2::Transform camera_to_marker = PoseToTransform(observation->pose.pose);
    if (camera_to_marker.getOrigin().length() > max_observation_distance_m_) {
      ROS_WARN_THROTTLE(2.0, "Ignoring visual observation beyond configured maximum distance");
      return;
    }

    const tf2::Transform uav_odom_to_base = PoseToTransform(uav_odom.pose.pose);
    const tf2::Transform ugv_odom_to_base = PoseToTransform(ugv_odom.pose.pose);
    const tf2::Transform ugv_odom_to_marker = ugv_odom_to_base * ugv_base_to_marker_;
    const tf2::Transform sample = uav_odom_to_base * uav_base_to_camera_ * camera_to_marker *
                                  ugv_odom_to_marker.inverse();
    AddSample(sample);
  }

  void AddSample(const tf2::Transform& sample) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (valid_ && IsOutlier(sample)) {
      ROS_WARN_THROTTLE(2.0, "Ignoring outlier frame-transform observation");
      return;
    }
    samples_.push_back(sample);
    constexpr std::size_t kMaximumSamples = 100;
    if (samples_.size() > kMaximumSamples) {
      samples_.erase(samples_.begin());
    }
    estimate_ = AverageSamples();
    valid_ = static_cast<int>(samples_.size()) >= minimum_observations_;
  }

  bool IsOutlier(const tf2::Transform& sample) const {
    const double translation_error = (sample.getOrigin() - estimate_.getOrigin()).length();
    const double rotation_error = sample.getRotation().angleShortestPath(estimate_.getRotation());
    return translation_error > max_sample_translation_error_m_ || rotation_error > max_sample_rotation_error_rad_;
  }

  tf2::Transform AverageSamples() const {
    tf2::Vector3 translation(0.0, 0.0, 0.0);
    tf2::Quaternion reference = samples_.front().getRotation().normalized();
    tf2::Quaternion rotation_sum(0.0, 0.0, 0.0, 0.0);
    for (const tf2::Transform& sample : samples_) {
      translation += sample.getOrigin();
      tf2::Quaternion rotation = sample.getRotation().normalized();
      if (rotation.dot(reference) < 0.0) {
        rotation = tf2::Quaternion(-rotation.x(), -rotation.y(), -rotation.z(), -rotation.w());
      }
      rotation_sum += rotation;
    }
    translation /= static_cast<double>(samples_.size());
    rotation_sum.normalize();
    return tf2::Transform(rotation_sum, translation);
  }

  void PublishTimerCallback(const ros::TimerEvent&) {
    tf2::Transform estimate;
    bool valid;
    std::size_t count;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      estimate = estimate_;
      valid = valid_;
      count = samples_.size();
    }

    std_msgs::Bool valid_message;
    valid_message.data = valid;
    valid_pub_.publish(valid_message);
    std_msgs::UInt32 count_message;
    count_message.data = static_cast<uint32_t>(count);
    count_pub_.publish(count_message);
    if (!valid) {
      return;
    }

    const ros::Time stamp = ros::Time::now();
    const geometry_msgs::TransformStamped transform =
        MakeTransformStamped(estimate, uav_odom_frame_, ugv_odom_frame_, stamp);
    dynamic_broadcaster_.sendTransform(transform);
    transform_pub_.publish(transform);

    geometry_msgs::PoseWithCovarianceStamped fused_pose;
    fused_pose.header.stamp = stamp;
    fused_pose.header.frame_id = uav_odom_frame_;
    fused_pose.pose.pose.position.x = estimate.getOrigin().x();
    fused_pose.pose.pose.position.y = estimate.getOrigin().y();
    fused_pose.pose.pose.position.z = estimate.getOrigin().z();
    fused_pose.pose.pose.orientation = tf2::toMsg(estimate.getRotation());
    fused_pose_pub_.publish(fused_pose);
  }

  ros::Time ValidStamp(const ros::Time& stamp) const {
    return stamp.isZero() ? ros::Time::now() : stamp;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber uav_odom_sub_;
  ros::Subscriber ugv_odom_sub_;
  ros::Subscriber observation_sub_;
  ros::Publisher transform_pub_;
  ros::Publisher fused_pose_pub_;
  ros::Publisher valid_pub_;
  ros::Publisher count_pub_;
  ros::Timer publish_timer_;
  tf2_ros::TransformBroadcaster dynamic_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  std::mutex mutex_;
  nav_msgs::Odometry uav_odom_;
  nav_msgs::Odometry ugv_odom_;
  bool has_uav_odom_{false};
  bool has_ugv_odom_{false};
  bool valid_{false};
  std::vector<tf2::Transform> samples_;
  tf2::Transform estimate_;
  tf2::Transform uav_base_to_camera_;
  tf2::Transform ugv_base_to_marker_;
  tf2::Transform ugv_base_to_camera_;
  std::string uav_odom_topic_;
  std::string ugv_odom_topic_;
  std::string observation_topic_;
  std::string uav_odom_frame_;
  std::string uav_base_frame_;
  std::string camera_frame_;
  std::string ugv_odom_frame_;
  std::string ugv_base_frame_;
  std::string ugv_marker_frame_;
  std::string ugv_camera_frame_;
  double sync_slop_sec_{0.10};
  int minimum_observations_{20};
  double max_observation_distance_m_{8.0};
  double max_sample_translation_error_m_{0.50};
  double max_sample_rotation_error_rad_{0.35};
  double publish_rate_hz_{20.0};
};

}  // namespace air_ground_coordinate_transform

int main(int argc, char** argv) {
  ros::init(argc, argv, "coordinate_transform");
  air_ground_coordinate_transform::CoordinateTransformNode node;
  ros::spin();
  return 0;
}
