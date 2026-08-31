#include <cmath>
#include <functional>
#include <memory>
#include <random>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

namespace gazebo {

class RelativePoseSensorPlugin : public ModelPlugin {
 public:
  RelativePoseSensorPlugin() : rng_(std::random_device{}()) {}

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = std::move(model);
    world_ = model_->GetWorld();
    camera_model_name_ = GetParam<std::string>(sdf, "camera_model_name", "iris_0");
    camera_link_name_ = GetParam<std::string>(sdf, "camera_link_name", "D435i::link");
    marker_link_name_ = GetParam<std::string>(sdf, "marker_link_name", "fiducial");
    camera_frame_ = GetParam<std::string>(sdf, "camera_frame", "iris_0/camera_link");
    observation_topic_ = GetParam<std::string>(sdf, "observation_topic", "/iris_0/ugv_observation");
    update_rate_ = GetParam<double>(sdf, "update_rate", 15.0);
    max_distance_ = GetParam<double>(sdf, "max_distance", 8.0);
    horizontal_fov_ = GetParam<double>(sdf, "horizontal_fov", 1.047198);
    vertical_fov_ = GetParam<double>(sdf, "vertical_fov", 0.785398);
    position_stddev_ = GetParam<double>(sdf, "position_stddev", 0.0);
    orientation_stddev_ = GetParam<double>(sdf, "orientation_stddev", 0.0);

    if (!ros::isInitialized()) {
      gzerr << "RelativePoseSensorPlugin requires Gazebo to be started by ROS.\n";
      return;
    }
    ros_node_.reset(new ros::NodeHandle());
    observation_pub_ = ros_node_->advertise<geometry_msgs::PoseWithCovarianceStamped>(observation_topic_, 1);
    marker_link_ = model_->GetLink(marker_link_name_);
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&RelativePoseSensorPlugin::OnUpdate, this));
  }

 private:
  template <typename T>
  T GetParam(const sdf::ElementPtr& sdf, const std::string& name, const T& default_value) const {
    return sdf->HasElement(name) ? sdf->Get<T>(name) : default_value;
  }

  bool ResolveCameraLink() {
    if (!camera_model_) {
      camera_model_ = world_->ModelByName(camera_model_name_);
    }
    if (camera_model_ && !camera_link_) {
      camera_link_ = camera_model_->GetLink(camera_link_name_);
    }
    return camera_link_ && marker_link_;
  }

  void OnUpdate() {
    if (!ros_node_ || !ResolveCameraLink()) {
      return;
    }

    const common::Time now = world_->SimTime();
    if (update_rate_ > 0.0 && (now - last_publish_time_).Double() < 1.0 / update_rate_) {
      return;
    }

    ignition::math::Pose3d relative = camera_link_->WorldPose().Inverse() * marker_link_->WorldPose();
    const ignition::math::Vector3d& position = relative.Pos();
    const double distance = position.Length();
    const double horizontal_angle = std::atan2(position.Y(), position.X());
    const double vertical_angle = std::atan2(position.Z(), std::hypot(position.X(), position.Y()));
    if (position.X() <= 0.0 || distance > max_distance_ ||
        std::abs(horizontal_angle) > horizontal_fov_ * 0.5 ||
        std::abs(vertical_angle) > vertical_fov_ * 0.5) {
      return;
    }

    ApplyNoise(&relative);
    geometry_msgs::PoseWithCovarianceStamped message;
    message.header.stamp = ros::Time(now.sec, now.nsec);
    message.header.frame_id = camera_frame_;
    message.pose.pose.position.x = relative.Pos().X();
    message.pose.pose.position.y = relative.Pos().Y();
    message.pose.pose.position.z = relative.Pos().Z();
    message.pose.pose.orientation.x = relative.Rot().X();
    message.pose.pose.orientation.y = relative.Rot().Y();
    message.pose.pose.orientation.z = relative.Rot().Z();
    message.pose.pose.orientation.w = relative.Rot().W();
    const double position_variance = position_stddev_ * position_stddev_;
    const double orientation_variance = orientation_stddev_ * orientation_stddev_;
    message.pose.covariance[0] = position_variance;
    message.pose.covariance[7] = position_variance;
    message.pose.covariance[14] = position_variance;
    message.pose.covariance[21] = orientation_variance;
    message.pose.covariance[28] = orientation_variance;
    message.pose.covariance[35] = orientation_variance;
    observation_pub_.publish(message);
    last_publish_time_ = now;
  }

  void ApplyNoise(ignition::math::Pose3d* pose) {
    if (position_stddev_ > 0.0) {
      std::normal_distribution<double> position_noise(0.0, position_stddev_);
      pose->Pos() += ignition::math::Vector3d(position_noise(rng_), position_noise(rng_), position_noise(rng_));
    }
    if (orientation_stddev_ > 0.0) {
      std::normal_distribution<double> orientation_noise(0.0, orientation_stddev_);
      ignition::math::Vector3d rpy = pose->Rot().Euler();
      pose->Rot() = ignition::math::Quaterniond(
          rpy.X() + orientation_noise(rng_), rpy.Y() + orientation_noise(rng_), rpy.Z() + orientation_noise(rng_));
    }
  }

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::ModelPtr camera_model_;
  physics::LinkPtr camera_link_;
  physics::LinkPtr marker_link_;
  event::ConnectionPtr update_connection_;
  std::unique_ptr<ros::NodeHandle> ros_node_;
  ros::Publisher observation_pub_;
  common::Time last_publish_time_;
  std::mt19937 rng_;
  std::string camera_model_name_;
  std::string camera_link_name_;
  std::string marker_link_name_;
  std::string camera_frame_;
  std::string observation_topic_;
  double update_rate_{15.0};
  double max_distance_{8.0};
  double horizontal_fov_{1.047198};
  double vertical_fov_{0.785398};
  double position_stddev_{0.0};
  double orientation_stddev_{0.0};
};

GZ_REGISTER_MODEL_PLUGIN(RelativePoseSensorPlugin)

}  // namespace gazebo
