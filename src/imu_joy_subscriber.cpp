#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Geometry>

#include "tf2_ros/transform_broadcaster.h"




class ImuJoySubscriber : public rclcpp::Node
{
public:
  ImuJoySubscriber()
  : Node("imu_joy_subscriber")
  {
    const double sqrt2h = std::sqrt(2.0) / 2.0;
    if (declare_parameter<bool>("use_threejs_coords", true)) {
      world_T_three_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    } else {
      world_T_three_ = Eigen::Quaterniond(sqrt2h, sqrt2h, 0.0, 0.0);
    }
    base_name_ = declare_parameter<std::string>("base_name", "world");
    marker_name_ = declare_parameter<std::string>("marker_name", "pose");
    v_max_ = declare_parameter("v_max", 1.0);
    declare_parameter<double>("initial_pose_x", 0.0);
    declare_parameter<double>("initial_pose_y", 0.0);
    declare_parameter<double>("initial_pose_z", 0.0);
    declare_parameter<double>("initial_pose_qx", 0.0);
    declare_parameter<double>("initial_pose_qy", 0.0);
    declare_parameter<double>("initial_pose_qz", 0.0);
    declare_parameter<double>("initial_pose_qw", 1.0);
    p_ = Eigen::Vector3d(get_parameter("initial_pose_x").as_double(),
			 get_parameter("initial_pose_y").as_double(),
			 get_parameter("initial_pose_z").as_double());
    q_init_ = Eigen::Quaterniond(get_parameter("initial_pose_qw").as_double(),
				 get_parameter("initial_pose_qx").as_double(),
				 get_parameter("initial_pose_qy").as_double(),
				 get_parameter("initial_pose_qz").as_double());
    v_ = Eigen::Vector3d::Zero();
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>
      (marker_name_, 10);

    auto imu_callback =
      [this](sensor_msgs::msg::Imu::UniquePtr msg) -> void {
	double now = this->get_clock()->now().seconds();
	double duration = prev_time_ == 0.0 ? 0.0 : now - prev_time_;
	prev_time_ = now;

	const Eigen::Quaterniond q(msg->orientation.w,
				   msg->orientation.x,
				   msg->orientation.y,
				   msg->orientation.z);
	// const Eigen::Vector3d x_axis(1, 0, 0);
	// const Eigen::Vector3d x_rotated = q * x_axis;
	// const Eigen::Vector3d y_axis(0, 1, 0);
	// const Eigen::Vector3d y_rotated = q * y_axis;
	q_ = q;

	auto v_base = q_ * v_;
	p_ += v_base * duration;

	Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
	iso.linear() = q_.toRotationMatrix();
	iso.translation() = p_;
	const auto iso_flipped = world_T_three_ * iso * q_init_;
	const auto q_flipped = Eigen::Quaterniond(iso_flipped.rotation());
	const auto p_flipped = iso_flipped.translation();
	auto pose_msg = geometry_msgs::msg::PoseStamped();
	pose_msg.header.stamp = msg->header.stamp;
	pose_msg.header.frame_id = base_name_;
	pose_msg.pose.position.x = p_flipped.x();
	pose_msg.pose.position.y = p_flipped.y();
	pose_msg.pose.position.z = p_flipped.z();
	pose_msg.pose.orientation.w = q_flipped.w();
	pose_msg.pose.orientation.x = q_flipped.x();
	pose_msg.pose.orientation.y = q_flipped.y();
	pose_msg.pose.orientation.z = q_flipped.z();
	pose_pub_->publish(pose_msg);
	// TF
	geometry_msgs::msg::TransformStamped tf_msg;
	tf_msg.header.stamp = msg->header.stamp;
	tf_msg.header.frame_id = base_name_;
	tf_msg.child_frame_id = marker_name_;
	tf_msg.transform.translation.x = p_flipped.x();
	tf_msg.transform.translation.y = p_flipped.y();
	tf_msg.transform.translation.z = p_flipped.z();
	tf_msg.transform.rotation.w = q_flipped.w();
	tf_msg.transform.rotation.x = q_flipped.x();
	tf_msg.transform.rotation.y = q_flipped.y();
	tf_msg.transform.rotation.z = q_flipped.z();
	tf_broadcaster_->sendTransform(tf_msg);
      };
    imu_sub_ =
      this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, imu_callback);
    auto joy_callback =
      [this](sensor_msgs::msg::Joy::UniquePtr msg) -> void {
	const double pan = (M_PI/2.0)*(1.0 - msg->axes[4])*0.5;
	double pan_sin, pan_cos;
	sincos(pan, &pan_sin, &pan_cos);
	const Eigen::Vector3d v(-msg->axes[0],
				pan_sin*msg->axes[1],
				-pan_cos*msg->axes[1]);
	v_ = v_max_ * v;
	v_max_ = get_parameter("v_max").as_double();
      };
    joy_sub_ =
      this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, joy_callback);
  }

private:
  double prev_time_ = 0.0;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  Eigen::Quaterniond q_;
  Eigen::Vector3d p_;
  Eigen::Vector3d v_;
  std::string base_name_;
  std::string marker_name_;
  double v_max_ = 1.0;
  Eigen::Quaterniond world_T_three_;
  Eigen::Quaterniond q_init_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuJoySubscriber>());
  rclcpp::shutdown();
  return 0;
}
