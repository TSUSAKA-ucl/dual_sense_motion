#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/empty.hpp"
#include <Eigen/Geometry>

#include "tf2_ros/transform_broadcaster.h"


const double IMU_DURATION = 0.005; // 200Hz
const double ACC_CUTOFF = 5.0; // seconds


class IncompleteIntegrator
{
private:  
  static std::pair<double, double> calc_coeff(double delta_t, double T)
  {	// use tustin's method
    const double eps = delta_t / (2.0 * T);
    const double a = (1.0 - eps) / (1.0 + eps);
    const double k = T;
    const double b = k * delta_t / (2.0 * T + delta_t);
    // const double a = (delta_t + 2.0 * T) / (delta_t - 2.0 * T);
    // const double b = delta_t * T / (delta_t - 2.0 * T);
    return {a, b};
  }
  const double a1_, b0_, b1_;
  double y_prev_ = 0.0;
  double u_prev_ = 0.0;
  double delta_t = IMU_DURATION;

public:
  IncompleteIntegrator(double delta_t, double T)
    : a1_(calc_coeff(delta_t, T).first)
    , b0_(calc_coeff(delta_t, T).second)
    , b1_(calc_coeff(delta_t, T).second)
    , delta_t(delta_t)
  { }
  double update(double u)
  {
    // const double y = -a1_ * y_prev_ + b0_ * u + b1_ * u_prev_;
    const double y = y_prev_ + delta_t * u;
    y_prev_ = y;
    u_prev_ = u;
    return y;
  }
  void reset()
  {
    y_prev_ = 0.0;
    u_prev_ = 0.0;
  }
};


class ImuJoySubscriber : public rclcpp::Node
{
public:
  ImuJoySubscriber()
  : Node("imu_joy_subscriber")
  , integ_x_(IMU_DURATION, ACC_CUTOFF)
  , integ_y_(IMU_DURATION, ACC_CUTOFF)
  , integ_z_(IMU_DURATION, ACC_CUTOFF)
  {
    const double sqrt2h = std::sqrt(2.0) / 2.0;
    if (declare_parameter<bool>("use_threejs_coords", true)) {
      world_T_three_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    } else {
      world_T_three_ = Eigen::Quaterniond(sqrt2h, sqrt2h, 0.0, 0.0);
    }
    use_acc_ = declare_parameter("use_acc", false);
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

    // API(py): https://docs.ros2.org/foxy/api/rclpy/api/services.html
    // API: https://docs.ros.org/en/ros2_packages/jazzy/api/rclcpp/generated/index.html
    auto reinit_acc_offset_client =
      this->create_client<std_srvs::srv::Empty>("reinitialize_acc_offset");
    auto reinit_orientation_client =
      this->create_client<std_srvs::srv::Empty>("reinitialize_orientation");
    while (!reinit_acc_offset_client->wait_for_service(std::chrono::seconds(3)) ||
	   !reinit_orientation_client->wait_for_service(std::chrono::seconds(3))) {
      if (!rclcpp::ok()) {
	RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the services. Exiting.");
	return;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
      
    // IMU
    auto imu_callback =
      [this](sensor_msgs::msg::Imu::UniquePtr msg) -> void {
	double now = this->get_clock()->now().seconds();
	double duration = prev_time_ == 0.0 ? 0.0 : now - prev_time_;
	if (duration >= IMU_DURATION * 1.5) {
	  RCLCPP_WARN(this->get_logger(),
		      "IMU message interval too long: %f sec", duration);
	}
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

	Eigen::Vector3d v_base;
	const double vx = integ_x_.update(msg->linear_acceleration.x);
	const double vy = integ_y_.update(msg->linear_acceleration.y);
	const double vz = integ_z_.update(msg->linear_acceleration.z);
	if (use_acc_) {
	  v_base = Eigen::Vector3d(vx, vy, vz);
	  RCLCPP_INFO(this->get_logger(),
		      "acc_x: %5.2f, acc_y: %5.2f, acc_z: %5.2f",//  -> v_x: %f, v_y: %f, v_z: %f",
		      msg->linear_acceleration.x,
		      msg->linear_acceleration.y,
		      msg->linear_acceleration.z
	  // 	      v_base.x(), v_base.y(), v_base.z()
		      );
	} else {
	  v_base = q_ * v_;
	}
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

    // joystick
    auto joy_callback =
      [this,
       reinit_acc_offset_client, reinit_orientation_client, empty_request
       ](sensor_msgs::msg::Joy::UniquePtr msg) -> void {
	const double pan = (M_PI/2.0)*(1.0 - msg->axes[4])*0.5;
	double pan_sin, pan_cos;
	sincos(pan, &pan_sin, &pan_cos);
	const Eigen::Vector3d v(-msg->axes[0],
				pan_sin*msg->axes[1],
				-pan_cos*msg->axes[1]);
	if (!use_acc_) {
	  v_ = v_max_ * v;
	}
	v_max_ = get_parameter("v_max").as_double();
	// API: https://docs.ros.org/en/ros2_packages/jazzy/api/rclcpp/generated/classrclcpp_1_1Client.html
	if (msg->buttons[4] == 1 && msg->buttons[0] == 1) { // X ボタンでリセット
	  if (!promise_a_.has_value())
	      // 不要 !promise_a_.value().future.valid()) // 既に呼び出し中でなければ
	    promise_a_.emplace(reinit_acc_offset_client->async_send_request(empty_request));
	}
	if (msg->buttons[4] == 1 && msg->buttons[1] == 1) { // ○ ボタンでリセット
	  if (!promise_o_.has_value())
	    promise_o_.emplace(reinit_orientation_client->async_send_request(empty_request));
	}
	if (promise_a_.has_value()) {
	  auto& promise_a = this->promise_a_.value();
	  if (promise_a.future.valid() && 
	      promise_a.future.wait_for(std::chrono::seconds(0)) ==
	      std::future_status::ready) {
	    RCLCPP_INFO(this->get_logger(), "reinitialize_acc_offset service call succeeded");
	    promise_a.get();
	    promise_a_.reset();
	    integ_x_.reset(); integ_y_.reset(); integ_z_.reset();
	    p_ = Eigen::Vector3d::Zero();
	  }
	}
	if (promise_o_.has_value()) {
	  auto& promise_o = this->promise_o_.value();
	  if (promise_o.future.valid() &&
	      promise_o.future.wait_for(std::chrono::seconds(0)) ==
	      std::future_status::ready) {
	    RCLCPP_INFO(this->get_logger(), "reinitialize_orientation service call succeeded");
	    promise_o.get();
	    promise_o_.reset();
	  }
	}
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
  bool use_acc_ = false;
  Eigen::Quaterniond world_T_three_;
  Eigen::Quaterniond q_init_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reinit_acc_offset_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reinit_orientation_client_;
  std::optional<rclcpp::Client<std_srvs::srv::Empty>::FutureAndRequestId> promise_a_;
  std::optional<rclcpp::Client<std_srvs::srv::Empty>::FutureAndRequestId> promise_o_;
  IncompleteIntegrator integ_x_;
  IncompleteIntegrator integ_y_;
  IncompleteIntegrator integ_z_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuJoySubscriber>());
  rclcpp::shutdown();
  return 0;
}
