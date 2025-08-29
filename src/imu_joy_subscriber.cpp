#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Geometry>

class ImuJoySubscriber : public rclcpp::Node
{
public:
  ImuJoySubscriber()
  : Node("imu_joy_subscriber")
  {
    v_max_ = declare_parameter("v_max", 1.0);
    p_ = Eigen::Vector3d::Zero();
    v_ = Eigen::Vector3d::Zero();
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
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
	const double sqrt2h = 1.0/sqrt(2);
	// const Eigen::Quaterniond rot_x(sqrt2h, sqrt2h, 0.0, 0.0);
	const Eigen::Quaterniond rot(sqrt2h, sqrt2h, 0.0, 0.0);
	const auto iso_flipped = rot * iso;
	const auto q_flipped = Eigen::Quaterniond(iso_flipped.rotation());
	const auto p_flipped = iso_flipped.translation();
	auto pose_msg = geometry_msgs::msg::PoseStamped();
	pose_msg.header.stamp = msg->header.stamp;
	pose_msg.header.frame_id = "world";
	pose_msg.pose.position.x = p_flipped.x();
	pose_msg.pose.position.y = p_flipped.y();
	pose_msg.pose.position.z = p_flipped.z();
	pose_msg.pose.orientation.w = q_flipped.w();
	pose_msg.pose.orientation.x = q_flipped.x();
	pose_msg.pose.orientation.y = q_flipped.y();
	pose_msg.pose.orientation.z = q_flipped.z();
	pose_pub_->publish(pose_msg);
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
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  Eigen::Quaterniond q_;
  Eigen::Vector3d p_;
  Eigen::Vector3d v_;
  double v_max_ = 1.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuJoySubscriber>());
  rclcpp::shutdown();
  return 0;
}
